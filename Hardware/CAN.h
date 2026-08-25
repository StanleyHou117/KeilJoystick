#ifndef __CAN_H
#define __CAN_H

#include "stm32f10x.h"
#include <stdint.h>

// CANopen-style COB-ID Offsets
#define COB_ID_TPDO1   0x180  // Transmit PDO (Device -> Bus)
#define COB_ID_RPDO1   0x200  // Receive PDO (Bus -> Device)
#define COB_ID_TSDO    0x580  // Transmit SDO (Response)
#define COB_ID_RSDO    0x600  // Receive SDO (Request)

// Object Dictionary Indexes
#define OBJ_HALL_SENSORS    0x2000  // Sub 1: Hall1, Sub 2: Hall2
#define OBJ_LED_BRIGHTNESS  0x2001  // Sub 1-4: W, B, G, R
#define OBJ_KEY_STATE       0x2002  // Sub 0: Bitmask

void    CAN_Config(uint8_t node_id);
uint8_t CAN_Send_Msg(uint32_t id, uint8_t *msg, uint8_t len);
void    CAN_Process_Stack(uint8_t node_id);

// External references to data stored in main/hardware modules
extern uint16_t Hall_Values[2];
extern uint8_t  LED_Values[4];
extern uint8_t  Current_Keys;

#endif