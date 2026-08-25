#ifndef __CAN_H
#define __CAN_H

#include "stm32f10x.h"

// Set your own CANopen Node ID for this STM32 device (1 to 127)
#define CANOPEN_NODE_ID     19 

// Standard CANopen Pre-defined Connection Set COB-IDs
#define COB_ID_TPDO1        (0x180 + CANOPEN_NODE_ID)
#define COB_ID_TPDO2        (0x280 + CANOPEN_NODE_ID)

// Function Prototypes
void CAN_Peripheral_Init(void);
void CANopen_Transmit_EncoderData(uint16_t *pEncoderArray);

#define COB_ID_ENC_A    0x180   /* encoders 0..5, 10-bit packed, DLC 8 */
#define COB_ID_ENC_B    0x181   /* encoders 6..9, 10-bit packed, DLC 5 */
#define COB_ID_COMMAND  0x200   /* PC -> STM: [node_index, cmd]        */

void CAN_Peripheral_Init(void);
void CANopen_Transmit_EncoderData(uint16_t *pEncoderArray);

/* Command channel: set by RX ISR, consumed by main loop. */
extern volatile uint8_t  g_cmd_pending;
extern volatile uint8_t  g_cmd_node;
extern volatile uint8_t  g_cmd_code;

#endif /* __CAN_H */