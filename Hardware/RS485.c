#include "rs485.h"
#include "Delay.h"

/* Encoder channel -> RS-485 address map: 0x00, 0x04, 0x08, 0x0C, 0x10, 0x14.
 * All divisible by 4 -- low 2 bits of the command byte carry the command
 * (0=position, 1=turns, 2=extended). */
const uint8_t AMT21_Addresses[RS485_CHANNELS] =
    {0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x24};

#define RX_BUFFER_SIZE  16
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t rx_write_index = 0;
static volatile uint16_t rx_read_index = 0;
static volatile uint16_t rx_total_count = 0;   /* bytes since last request */
static volatile uint8_t  last_command_sent = 0;/* for echo detection when RE# tied low */

/* Set while DE is asserted (we are transmitting). Any byte the UART clocks
 * in during this window is our own echo or a line glitch and is dropped.
 * Once we're back in RX mode, real reply bytes are kept. */
static volatile uint8_t  tx_active = 0;

void RS485_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    /* PA8 - DE/RE control pin (tie DE and /RE together on the transceiver) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA9 - USART1 TX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA10 - USART1 RX.
     * Pulled up so the line idles high while the transceiver receiver is
     * briefly tri-stated during turnaround. A floating input here is what
     * generates a spurious start-bit glitch. */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RS485_RX_MODE();

    USART_Cmd(USART1, DISABLE);
    USART_DeInit(USART1);   /* clean re-init so baud can be changed at runtime */

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

void RS485_FlushRx(void)
{
    __disable_irq();
    rx_write_index = 0;
    rx_read_index = 0;
    rx_total_count = 0;
    __enable_irq();
}

uint16_t RS485_BytesReceived(void)
{
    return rx_total_count;
}

uint8_t RS485_ReadByte(uint8_t *pByte)
{
    if (rx_read_index == rx_write_index) return 0;
    *pByte = rx_buffer[rx_read_index];
    rx_read_index = (rx_read_index + 1) % RX_BUFFER_SIZE;
    return 1;
}

/**
 * @brief  Send the "read position" command (0x00) to an arbitrary address.
 *
 * tx_active gates the ISR: while DE is asserted we drop any received byte
 * (our own echo / line glitch). The instant TC is set we drop DE, flip to
 * RX, and clear tx_active so the encoder's reply is captured.
 */
void AMT21_RequestAtAddress(uint8_t address)
{
    /* Clear stale state and any pending overrun BEFORE we transmit. */
    RS485_FlushRx();
    (void)USART1->SR;
    (void)USART1->DR;                 /* clear RXNE/ORE if set */

    tx_active = 1;                    /* discard anything received now */
    RS485_TX_MODE();
    Delay_us(10);                     /* let DE/RE physically settle (per CUI) */

    /* Make sure TC is clear before we start, so the wait below is real. */
    USART_ClearFlag(USART1, USART_FLAG_TC);

    USART_SendData(USART1, address | 0x00);   /* 0x00 = read position */
    last_command_sent = (uint8_t)(address | 0x00);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    /* Release the bus, then reopen the receiver. Real reply bytes arriving
     * after this point are kept. */
    RS485_RX_MODE();
    tx_active = 0;
}

void AMT21_RequestPosition(uint8_t channel)
{
    if (channel >= RS485_CHANNELS) return;
    AMT21_RequestAtAddress(AMT21_Addresses[channel]);
}

/* AMT21 checksum: K1 (bit 15) = inverted XOR of odd bits 1..13,
 * K0 (bit 14) = inverted XOR of even bits 0..12. */
uint8_t AMT21_ChecksumOK(uint16_t msg)
{
    uint8_t k1, k0;
    uint8_t odd = 0, even = 0;
    int i;

    k1 = (msg & 0x8000) >> 15;
    k0 = (msg & 0x4000) >> 14;

    for (i = 0; i < 14; i++) {
        if (i % 2 == 0)
            even ^= ((msg >> i) & 1);
        else
            odd ^= ((msg >> i) & 1);
    }

    return (k1 == (uint8_t)!odd) && (k0 == (uint8_t)!even);
}

uint8_t AMT21_ReadResponse(uint16_t *pDestPosition)
{
    uint8_t low_byte, high_byte;
    uint16_t raw_msg;

    if (!RS485_ReadByte(&low_byte))  return 0;
    if (!RS485_ReadByte(&high_byte)) return 0;

    raw_msg = (uint16_t)((high_byte << 8) | low_byte);

    if (AMT21_ChecksumOK(raw_msg)) {
        *pDestPosition = raw_msg & 0x3FFF;   /* AMT213 = 14-bit */
        return 1;
    }
    return 0;
}

/**
 * @brief  Complete request/response transaction with timeout.
 * @retval 1 on success. On failure returns 0 and sets *pDestPosition to:
 * AMT21_ERR_TIMEOUT  (0xFFFF)  no bytes received at all
 * AMT21_ERR_CHECKSUM (0xFFFE)  bytes arrived but checksum failed
 */
uint8_t AMT21_ReadPosition(uint8_t channel, uint16_t *pDestPosition, uint32_t timeout_us)
{
    AMT21_RequestPosition(channel);

    while (timeout_us--)
    {
        if (rx_total_count >= 2)
        {
            if (AMT21_ReadResponse(pDestPosition))
                return 1;
            *pDestPosition = AMT21_ERR_CHECKSUM;
            return 0;
        }
        Delay_us(1);
    }

    *pDestPosition = (rx_total_count > 0) ? AMT21_ERR_CHECKSUM : AMT21_ERR_TIMEOUT;
    return 0;
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t received_byte = (uint8_t)USART_ReceiveData(USART1);

        /* Byte received while we were still transmitting -> echo/glitch, drop it. */
        if (tx_active)
        {
            USART_ClearITPendingBit(USART1, USART_IT_RXNE);
            return;
        }

        uint16_t next_write = (rx_write_index + 1) % RX_BUFFER_SIZE;

        rx_total_count++;
        if (next_write != rx_read_index)
        {
            rx_buffer[rx_write_index] = received_byte;
            rx_write_index = next_write;
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

/**
 * @brief  Send an AMT21 extended command to a channel.
 *         Sequence: <address + 0x02> then <cmd_byte>.
 *         cmd 1 -> 0x5E (set zero), cmd 2 -> 0x75 (reset).
 *         Encoder gives NO response and immediately resets; caller must
 *         wait for power-up before polling this channel again.
 */
void AMT21_SendExtendedCommand(uint8_t channel, uint8_t cmd)
{
    uint8_t addr, cmd_byte;

    if (channel >= RS485_CHANNELS) return;
    if (cmd == 1)      cmd_byte = 0x5E;   /* set zero */
    else if (cmd == 2) cmd_byte = 0x75;   /* reset    */
    else return;

    addr = (uint8_t)(AMT21_Addresses[channel] + 0x02);  /* extended-cmd modifier */

    RS485_FlushRx();
    (void)USART1->SR;
    (void)USART1->DR;

    tx_active = 1;
    RS485_TX_MODE();
    Delay_us(10);
    USART_ClearFlag(USART1, USART_FLAG_TC);

    /* Byte 1: address + 0x02 */
    USART_SendData(USART1, addr);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    /* Byte 2: the command */
    USART_SendData(USART1, cmd_byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    RS485_RX_MODE();
    tx_active = 0;
}