#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"
#include <stdint.h>

void LED_PWM_Init(void);
void LED_SetColor(uint8_t index, uint8_t brightness);

#endif