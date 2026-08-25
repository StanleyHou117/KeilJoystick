#include "CAN.h"
#include "LED.h"
#include "stm32f10x_can.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

// Global data buffers for the "Object Dictionary"
uint16_t Hall_Values[2] = {0, 0};
uint8_t  LED_Values[4]  = {0, 0, 0, 0};
uint8_t  Current_Keys   = 0;

void CAN_Config(uint8_t node_id) {
    GPIO_InitTypeDef gpio;
    CAN_InitTypeDef can;
    CAN_FilterInitTypeDef filter;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_11; // RX
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_12; // TX
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    can.CAN_ABOM = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_5tq;
    can.CAN_BS2 = CAN_BS2_3tq;
    can.CAN_Prescaler = 4; // 1Mbps @ 36MHz
    CAN_Init(CAN1, &can);

    // Filter 0: Listen for SDO (0x600 + NodeID)
    filter.CAN_FilterNumber = 0;
    filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh = (uint16_t)((COB_ID_RSDO + node_id) << 5);
    filter.CAN_FilterIdLow = 0x0000;
    filter.CAN_FilterMaskIdHigh = 0xFFFF << 5; 
    filter.CAN_FilterMaskIdLow = 0x0000;
    filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter);

    // Filter 1: Listen for RPDO1 (0x200 + NodeID)
    filter.CAN_FilterNumber = 1;
    filter.CAN_FilterIdHigh = (uint16_t)((COB_ID_RPDO1 + node_id) << 5);
    CAN_FilterInit(&filter);
}

uint8_t CAN_Send_Msg(uint32_t id, uint8_t *msg, uint8_t len) {
    CanTxMsg TxMessage;
    TxMessage.StdId = id;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = (len > 8) ? 8 : len;
    for(int i = 0; i < TxMessage.DLC; i++) TxMessage.Data[i] = msg[i];

    uint8_t mbox = CAN_Transmit(CAN1, &TxMessage);
    uint16_t timeout = 0;
    while((CAN_TransmitStatus(CAN1, mbox) != CAN_TxStatus_Ok) && (timeout < 0xFFFF)) timeout++;
    return (timeout >= 0xFFFF) ? 1 : 0;
}

// Simulated SDO Response
void Handle_SDO_Request(uint8_t node_id, CanRxMsg *Rx) {
    uint8_t tx_buf[8] = {0};
    uint16_t index = (Rx->Data[2] << 8) | Rx->Data[1];
    uint8_t sub_index = Rx->Data[3];
    
    tx_buf[0] = 0x40; // Default Upload Response
    tx_buf[1] = Rx->Data[1]; 
    tx_buf[2] = Rx->Data[2];
    tx_buf[3] = sub_index;

    if (index == OBJ_HALL_SENSORS) {
        if (sub_index <= 2) {
            uint16_t val = Hall_Values[sub_index-1];
            tx_buf[4] = val & 0xFF;
            tx_buf[5] = (val >> 8) & 0xFF;
        }
    } else if (index == OBJ_LED_BRIGHTNESS) {
        if (sub_index >= 1 && sub_index <= 4) {
            tx_buf[4] = LED_Values[sub_index-1];
        }
    } else if (index == OBJ_KEY_STATE) {
        tx_buf[4] = Current_Keys;
    }
    
    CAN_Send_Msg(COB_ID_TSDO + node_id, tx_buf, 8);
}

void CAN_Process_Stack(uint8_t node_id) {
    CanRxMsg RxMessage;
    if (CAN_MessagePending(CAN1, CAN_Filter_FIFO0) == 0) return;

    CAN_Receive(CAN1, CAN_Filter_FIFO0, &RxMessage);

    // Logic for RPDO1 (Direct LED Control)
    if (RxMessage.StdId == (COB_ID_RPDO1 + node_id)) {
        uint8_t channel = RxMessage.Data[0];
        uint8_t brightness = RxMessage.Data[1];
        if (channel >= 1 && channel <= 4) {
            LED_Values[channel-1] = brightness;
            LED_SetColor(channel, brightness);
        }
    }
    // Logic for SDO
    else if (RxMessage.StdId == (COB_ID_RSDO + node_id)) {
        Handle_SDO_Request(node_id, &RxMessage);
    }
}