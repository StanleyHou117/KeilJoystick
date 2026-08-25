#include "LED.h"

void LED_PWM_Init(void) {
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    // 1kHz PWM Frequency
    tim.TIM_Period = 100 - 1; 
    tim.TIM_Prescaler = 720 - 1; 
    tim.TIM_ClockDivision = 0;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &tim);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;

    TIM_OC1Init(TIM4, &oc); // PB6 - White
    TIM_OC2Init(TIM4, &oc); // PB7 - Blue
    TIM_OC3Init(TIM4, &oc); // PB8 - Green
    TIM_OC4Init(TIM4, &oc); // PB9 - Red

    TIM_Cmd(TIM4, ENABLE);
}

void LED_SetColor(uint8_t index, uint8_t brightness) {
    // Turn all off first
    TIM_SetCompare1(TIM4, 0);
    TIM_SetCompare2(TIM4, 0);
    TIM_SetCompare3(TIM4, 0);
    TIM_SetCompare4(TIM4, 0);

    if (brightness > 100) brightness = 100;

    switch(index) {
        case 1: TIM_SetCompare1(TIM4, brightness); break; // White
        case 2: TIM_SetCompare2(TIM4, brightness); break; // Blue
        case 3: TIM_SetCompare3(TIM4, brightness); break; // Green
        case 4: TIM_SetCompare4(TIM4, brightness); break; // Red
    }
}