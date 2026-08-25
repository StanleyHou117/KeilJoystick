#include "Key.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

void Key_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin  = GPIO_Pin_1 | GPIO_Pin_2; // PA1, PA2
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1; // PB0, PB1
    GPIO_Init(GPIOB, &gpio);
}

uint8_t Key_Read(void)
{
    uint8_t state = 0;
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == Bit_RESET) state |= 0x01;
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_RESET) state |= 0x02;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET) state |= 0x04;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == Bit_RESET) state |= 0x08;
    return state;
}