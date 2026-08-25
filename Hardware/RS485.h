#ifndef __RS485_H
#define __RS485_H

#include "stm32f10x.h"

#define RS485_CHANNELS  10


/* Error markers reported over CAN in place of a position.
 * Both are > 0x3FFF so they can never collide with a real 14-bit value. */
#define AMT21_ERR_TIMEOUT   0xFFFF   /* no bytes received at all          */
#define AMT21_ERR_CHECKSUM  0xFFFE   /* bytes received but checksum failed */

/* RS485 Mode Control Macros.
 * HARDWARE: RE# (SP3485 pin 2) is tied permanently to GND -> receiver is
 * ALWAYS enabled, so RO always actively drives PA10 (fixes the floating-RO
 * RC-decay seen on the scope). PA8 now controls DE ONLY:
 * PA8 HIGH -> DE on  -> we drive the bus (transmit)
 * PA8 LOW  -> DE off -> bus released; encoder can reply
 * We hear our own transmitted byte as an echo, but RX is flushed before
 * the reply is read, so it is harmless. */
#define RS485_TX_MODE()   GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define RS485_RX_MODE()   GPIO_ResetBits(GPIOA, GPIO_Pin_8)

/* AMT21 node addresses (must be divisible by 4).
 * Channel 0 = 0x54 = factory default. See table in RS485.c. */
extern const uint8_t AMT21_Addresses[RS485_CHANNELS];

void     RS485_Init(uint32_t baudrate);
void     RS485_FlushRx(void);
uint16_t RS485_BytesReceived(void);
uint8_t  RS485_ReadByte(uint8_t *pByte);

void     AMT21_RequestAtAddress(uint8_t address);
void     AMT21_RequestPosition(uint8_t channel);
uint8_t  AMT21_ChecksumOK(uint16_t msg);
uint8_t  AMT21_ReadResponse(uint16_t *pDestPosition);
uint8_t  AMT21_ReadPosition(uint8_t channel, uint16_t *pDestPosition, uint32_t timeout_us);
/* Extended commands. cmd: 1 = set zero (0x5E), 2 = reset (0x75). */
void AMT21_SendExtendedCommand(uint8_t channel, uint8_t cmd);

#endif /* __RS485_H */