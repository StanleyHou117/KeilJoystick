#include "MCP3202.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

// --- Configuration Constants ---
// Filter smoothing factor alpha (Value between 0.0 and 1.0)
// Lower alpha = smoother graph, but adds slight tracking delay.
#define FILTER_ALPHA    1.0f

// High-side saturation threshold. Readings at or above this value 
// are locked to 4095 to eliminate power-rail ripple noise.
#define SATURATION_LOCK 4095

// --- Filter State Memory ---
static float filtered_ch0 = -1.0f;
static float filtered_ch1 = -1.0f;

/**
 * Initializes GPIO Pins and SPI1 Peripheral for the MCP3202 ADC
 */
void MCP3202_Init(void) {
    GPIO_InitTypeDef gpio;
    SPI_InitTypeDef  spi;

    // Enable Clocks for GPIOA and SPI1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

    // PA4: CS (Output Push-Pull)
    gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    GPIO_SetBits(GPIOA, GPIO_Pin_4); // Idle High (Deselected)

    // PA5: SCK, PA7: MOSI (Alternate Function Push-Pull)
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    // PA6: MISO (Floating Input)
    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    // SPI1 Configuration
    SPI_StructInit(&spi);
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_Low;      // Clock idle low
    spi.SPI_CPHA = SPI_CPHA_1Edge;    // Sample on first edge
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // Approx 1MHz for stability
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    
    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);
    
    // Clear/Reset filter history
    filtered_ch0 = -1.0f;
    filtered_ch1 = -1.0f;
}

/**
 * Internal SPI Transmit/Receive Wrapper Function
 */
static uint8_t SPI_RW(uint8_t data) {
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI1, data);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(SPI1);
}

/**
 * Reads a channel from the MCP3202, applies high-side lock, and filters data
 * @param channel: 0 for CH0, 1 for CH1
 * @return 12-bit filtered and locked ADC reading (0 - 4095)
 */
uint16_t MCP3202_Read(uint8_t channel) {
    uint8_t byte2, byte3;
    uint16_t raw_result;

    /* MCP3202 Command Bit Protocol:
       Byte 1: [0 0 0 0 0 0 0 1] -> The '1' is the START bit.
       Byte 2: [SGL/DIFF][ODD/SIGN][MSBF][X][X][X][X][X]
               SGL/DIFF = 1 (Single-Ended)
               ODD/SIGN = channel (0 or 1)
               MSBF     = 1 (MSB First)
    */
    uint8_t cmd = 0xA0 | ((channel & 0x01) << 6); 

    GPIO_ResetBits(GPIOA, GPIO_Pin_4); // Select chip (CS Low)

    SPI_RW(0x01);             // Send Start Bit
    byte2 = SPI_RW(cmd);      // Send Config, receive MSB bits
    byte3 = SPI_RW(0x00);     // Send dummy byte, receive LSB bits

    GPIO_SetBits(GPIOA, GPIO_Pin_4);   // Deselect chip (CS High)

    // Reconstruct the 12-bit raw reading from the SPI stream
    raw_result = ((uint16_t)(byte2 & 0x0F) << 8) | byte3;

    // --- 1. FIRMWARE LOCK / DEADZONE CLAMP ---
    // If the raw signal drops into the noisy power rail margin near the top,
    // lock it directly to the 12-bit maximum ceiling (4095).
    if (raw_result >= SATURATION_LOCK) {
        raw_result = SATURATION_LOCK;
    }

    // --- 2. EXPONENTIAL MOVING AVERAGE FILTER ---
    if (channel == 0) {
        // If it's the first sample, initialize history to prevent a slow ramp up from 0
        if (filtered_ch0 < 0.0f) {
            filtered_ch0 = (float)raw_result;
        } else {
            filtered_ch0 = (FILTER_ALPHA * (float)raw_result) + ((1.0f - FILTER_ALPHA) * filtered_ch0);
        }
        return (uint16_t)(filtered_ch0 + 0.5f); // Return rounded integer
    } 
    else {
        if (filtered_ch1 < 0.0f) {
            filtered_ch1 = (float)raw_result;
        } else {
            filtered_ch1 = (FILTER_ALPHA * (float)raw_result) + ((1.0f - FILTER_ALPHA) * filtered_ch1);
        }
        return (uint16_t)(filtered_ch1 + 0.5f); // Return rounded integer
    }
}