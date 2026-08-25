#ifndef __MCP3202_H
#define __MCP3202_H

#include "stm32f10x.h"

/**
 * @brief  Initializes SPI1 and GPIO pins for MCP3202.
 * Pins used: PA4 (CS), PA5 (SCK), PA6 (MISO), PA7 (MOSI)
 */
void MCP3202_Init(void);

/**
 * @brief  Reads a 12-bit value from the specified channel.
 * @param  channel: 0 for CH0, 1 for CH1.
 * @retval 12-bit ADC value (0-4095).
 */
uint16_t MCP3202_Read(uint8_t channel);

#endif