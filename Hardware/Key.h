#ifndef __KEY_H__
#define __KEY_H__

#include <stdint.h>

void    Key_Init(void);
uint8_t Key_Read(void); // Returns bitmask: Bit0=PB0, Bit1=PB1, Bit2=PA1, Bit3=PA2

#endif