#include "can.h"
#include "rs485.h"   /* <-- add this: for RS485_CHANNELS */

/**
  * @brief  Initializes PA11(RX) and PA12(TX) for CAN communication at 1Mbps.
  * @note   Assumes a 36MHz APB1 Peripheral Clock (Standard for 72MHz Core).
  */
void CAN_Peripheral_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    // 1. Enable Clocks for GPIOA and CAN1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    // 2. Configure PA11 (CAN_RX) as Floating Input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. Configure PA12 (CAN_TX) as Alternate Function Push-Pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 4. Initialize CAN Cell Base Parameters
    CAN_DeInit(CAN1);
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;  // Automatic Bus-Off Management
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE; // Automatic retransmission if error occurs
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

    /* 5. CAN Timing Configuration for 1 Mbps
       Formula: BaudRate = APB1_CLK / (BRP * (1 + TS1 + TS2))
       Given APB1 = 36 MHz:
       36MHz / (4 * (1 + 5 + 3)) = 36MHz / (4 * 9) = 1 Mbps
       Sample Point = (1 + TS1) / (1 + TS1 + TS2) = 6/9 = 66.6% (Standard for CANopen)
    */
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_5tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
    CAN_InitStructure.CAN_Prescaler = 4;
    CAN_Init(CAN1, &CAN_InitStructure);

    // 6. CAN Filter Initialization (Pass-All Configuration for simplicity)
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
		
		
		    /* Enable FIFO0 message-pending interrupt for the command channel. */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
		
		
}
/* Command-channel shared state. */
volatile uint8_t g_cmd_pending = 0;
volatile uint8_t g_cmd_node    = 0;
volatile uint8_t g_cmd_code    = 0;

/* Pack n 10-bit values, little-endian bit order, into out[]. */
static void pack10(const uint16_t *vals, uint8_t n, uint8_t *out)
{
    uint32_t bitpos = 0;
    uint8_t i, b;
    uint8_t nbytes = (uint8_t)((n * 10 + 7) / 8);

    for (i = 0; i < nbytes; i++) out[i] = 0;

    for (i = 0; i < n; i++)
    {
        uint16_t raw = vals[i];
        uint16_t v;
        if (raw == 0xFFFF || raw == 0xFFFE)   /* AMT21_ERR_TIMEOUT / _CHECKSUM */
            v = 0x3FF;                        /* invalid sentinel */
        else
            v = (uint16_t)((raw & 0x3FFF) >> 4);  /* 14-bit -> top 10 bits */

        for (b = 0; b < 10; b++)
        {
            if (v & (1u << b))
            {
                uint32_t bp = bitpos + b;
                out[bp >> 3] |= (uint8_t)(1u << (bp & 7));
            }
        }
        bitpos += 10;
    }
}

static void CAN_TxBlocking(uint16_t id, uint8_t *data, uint8_t dlc)
{
    CanTxMsg TxMessage;
    uint8_t mbox, i;

    TxMessage.StdId = id;
    TxMessage.IDE   = CAN_Id_Standard;
    TxMessage.RTR   = CAN_RTR_Data;
    TxMessage.DLC   = dlc;
    for (i = 0; i < dlc; i++) TxMessage.Data[i] = data[i];

    mbox = CAN_Transmit(CAN1, &TxMessage);
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) ||
           (CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_NoMailBox));
}

void CANopen_Transmit_EncoderData(uint16_t *pEncoderArray)
{
    uint8_t frameA[8];
    uint8_t frameB[8];

    pack10(&pEncoderArray[0], 6, frameA);   /* enc 0..5 -> 8 bytes */
    pack10(&pEncoderArray[6], 4, frameB);   /* enc 6..9 -> 5 bytes */

    CAN_TxBlocking(COB_ID_ENC_A, frameA, 8);
    CAN_TxBlocking(COB_ID_ENC_B, frameB, 5);
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) == RESET) return;

    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

    if (RxMessage.StdId == COB_ID_COMMAND && RxMessage.DLC >= 2)
    {
        uint8_t node = RxMessage.Data[0];
        uint8_t code = RxMessage.Data[1];
        if (node < RS485_CHANNELS && (code == 1 || code == 2))
        {
            g_cmd_node    = node;
            g_cmd_code    = code;
            g_cmd_pending = 1;   /* main loop executes it safely */
        }
    }
    /* CAN_Receive already released the FIFO; flag clears automatically. */
}