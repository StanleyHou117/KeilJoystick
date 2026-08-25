#include "stm32f10x.h"
#include "rs485.h"
#include "can.h"
#include "Delay.h"

/* ===========================================================================
 *  MODE SELECT
 *  1 = DIAGNOSTIC ADDRESS/BAUD SCANNER
 *  0 = NORMAL BRIDGE: polls encoders, transmits encoder frames.
 * ======================================================================== */
#define DIAG_ADDRESS_SCAN   0

/* Set to 1 to bypass ALL AMT21 parsing and dump raw RX bytes to CAN 0x100.
 * Leave at 0 for normal operation. */
#define DIAG_RAW_PASSTHROUGH  0

/* RS-485 baud rate. Must match the encoder's AMT Viewpoint setting. */
#define RS485_BAUD          115200UL

/* Scope trigger (PB0) */
#define SCOPE_TRIG_PORT    GPIOB
#define SCOPE_TRIG_PIN     GPIO_Pin_0

/* Normal-mode settings: all six encoders active. */
#define ACTIVE_CHANNELS    10      /* max 6 */

/* Per-read timeout. At 115200 the 2-byte reply takes ~175 us. */
#define AMT21_TIMEOUT_US   2000

/* Diagnostic CAN IDs (unused in normal mode) */
#define COB_ID_SCAN_HIT    0x555
#define COB_ID_SCAN_DONE   0x556

void Scope_Trigger_Pin_Init(void);

#if DIAG_ADDRESS_SCAN
/* ---------------------------------------------------------------------- */

static const uint32_t scan_bauds[] = {9600, 19200, 115200, 2000000};
#define N_BAUDS  (sizeof(scan_bauds) / sizeof(scan_bauds[0]))

static void CAN_SendRaw(uint16_t std_id, uint8_t *data, uint8_t dlc)
{
    CanTxMsg msg;
    uint8_t mbox, i;

    msg.StdId = std_id;
    msg.IDE   = CAN_Id_Standard;
    msg.RTR   = CAN_RTR_Data;
    msg.DLC   = dlc;
    for (i = 0; i < dlc; i++) msg.Data[i] = data[i];

    mbox = CAN_Transmit(CAN1, &msg);
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) ||
           (CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_NoMailBox));
}

int main(void)
{
    uint8_t  baud_idx;
    uint16_t addr;
    uint8_t  hits_total;

    SystemInit();
    Scope_Trigger_Pin_Init();
    CAN_Peripheral_Init();

    while (1)
    {
        hits_total = 0;

        for (baud_idx = 0; baud_idx < N_BAUDS; baud_idx++)
        {
            RS485_Init(scan_bauds[baud_idx]);
            Delay_ms(5);

            for (addr = 0x00; addr <= 0xFC; addr += 4)
            {
                uint8_t b0 = 0, b1 = 0, n, csum_ok = 0;
                uint16_t raw = 0;
                uint8_t frame[8];

                GPIO_SetBits(SCOPE_TRIG_PORT, SCOPE_TRIG_PIN);
                AMT21_RequestAtAddress((uint8_t)addr);
                GPIO_ResetBits(SCOPE_TRIG_PORT, SCOPE_TRIG_PIN);

                Delay_ms(4);

                n = (uint8_t)RS485_BytesReceived();
                if (n == 0)
                    continue;

                RS485_ReadByte(&b0);
                RS485_ReadByte(&b1);
                raw = (uint16_t)((b1 << 8) | b0);
                csum_ok = AMT21_ChecksumOK(raw);
                hits_total++;

                frame[0] = baud_idx;
                frame[1] = (uint8_t)addr;
                frame[2] = n;
                frame[3] = b0;
                frame[4] = b1;
                frame[5] = csum_ok;
                frame[6] = (uint8_t)(raw & 0xFF);
                frame[7] = (uint8_t)((raw >> 8) & 0x3F);
                CAN_SendRaw(COB_ID_SCAN_HIT, frame, 8);
            }
        }

        {
            uint8_t done[1];
            done[0] = hits_total;
            CAN_SendRaw(COB_ID_SCAN_DONE, done, 1);
        }

        Delay_ms(500);
    }
}

#elif DIAG_RAW_PASSTHROUGH  /* ======= RAW RS-485 -> CAN DUMP ============= */

#define COB_ID_RAW   0x100

static void CAN_SendRaw(uint16_t std_id, uint8_t *data, uint8_t dlc)
{
    CanTxMsg msg;
    uint8_t mbox, i;

    msg.StdId = std_id;
    msg.IDE   = CAN_Id_Standard;
    msg.RTR   = CAN_RTR_Data;
    msg.DLC   = dlc;
    for (i = 0; i < dlc; i++) msg.Data[i] = data[i];

    mbox = CAN_Transmit(CAN1, &msg);
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) ||
           (CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_NoMailBox));
}

int main(void)
{
    SystemInit();
    Scope_Trigger_Pin_Init();
    RS485_Init(RS485_BAUD);
    CAN_Peripheral_Init();

    while (1)
    {
        uint8_t frame[8];
        uint8_t n, i;
        uint8_t b;

        GPIO_SetBits(SCOPE_TRIG_PORT, SCOPE_TRIG_PIN);
        AMT21_RequestPosition(0);
        GPIO_ResetBits(SCOPE_TRIG_PORT, SCOPE_TRIG_PIN);

        Delay_ms(2);

        n = (uint8_t)RS485_BytesReceived();

        frame[0] = n;
        for (i = 0; i < 7; i++)
            frame[i + 1] = RS485_ReadByte(&b) ? b : 0x00;

        CAN_SendRaw(COB_ID_RAW, frame, 8);

        Delay_ms(20);
    }
}

#else  /* ================== NORMAL BRIDGE MODE ========================== */
static uint16_t encoder_positions[RS485_CHANNELS];

int main(void)
{
    uint8_t ch;

    SystemInit();
    Scope_Trigger_Pin_Init();
    RS485_Init(RS485_BAUD);
    CAN_Peripheral_Init();

    for (ch = 0; ch < RS485_CHANNELS; ch++)
        encoder_positions[ch] = AMT21_ERR_TIMEOUT;

    while (1)
    {
        /* Execute a pending PC command at a safe (non-mid-poll) point. */
        if (g_cmd_pending)
        {
            uint8_t node = g_cmd_node, code = g_cmd_code;
            g_cmd_pending = 0;
            AMT21_SendExtendedCommand(node, code);
            Delay_ms(300);   /* block while the encoder reboots */
        }

        GPIO_SetBits(SCOPE_TRIG_PORT, SCOPE_TRIG_PIN);

        for (ch = 0; ch < RS485_CHANNELS; ch++)
        {
            uint16_t pos;
            AMT21_ReadPosition(ch, &pos, AMT21_TIMEOUT_US);
            encoder_positions[ch] = pos;
        }

        GPIO_ResetBits(SCOPE_TRIG_PORT, SCOPE_TRIG_PIN);

        CANopen_Transmit_EncoderData(encoder_positions);
        Delay_ms(100);
    }
}

#endif /* DIAG_ADDRESS_SCAN */

void Scope_Trigger_Pin_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = SCOPE_TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SCOPE_TRIG_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(SCOPE_TRIG_PORT, SCOPE_TRIG_PIN);
}