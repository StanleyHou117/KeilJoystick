#include "stm32f10x.h"
#include "CAN.h"
#include "Key.h"
#include "LED.h"
#include "MCP3202.h"
#include "Delay.h"

#define MY_NODE_ID 15

int main(void)
{
    // 1. Hardware Initialization
    SystemInit();
    CAN_Config(MY_NODE_ID); // 1Mbps
    Key_Init();
    LED_PWM_Init();
    MCP3202_Init();

    uint8_t tpdo_data[8];

    while (1)
    {
        // 2. Update Sensor Data (The Object Dictionary)
        Hall_Values[0] = MCP3202_Read(0); 
        Hall_Values[1] = MCP3202_Read(1); 
        Current_Keys   = Key_Read();

        // 3. Process Incoming CAN Messages
        // This updates the hardware and the internal state
        CAN_Process_Stack(MY_NODE_ID);

        // 4. Pack 8-Byte TPDO1
        // Bytes 0-3: Hall Sensors (Masked 12-bit per Protocol Spec)
        tpdo_data[0] = (uint8_t)(Hall_Values[0] & 0xFF);
        tpdo_data[1] = (uint8_t)((Hall_Values[0] >> 8) & 0x0F);
        tpdo_data[2] = (uint8_t)(Hall_Values[1] & 0xFF);
        tpdo_data[3] = (uint8_t)((Hall_Values[1] >> 8) & 0x0F);
        
        // Byte 4: Key State (Bitmask)
        tpdo_data[4] = Current_Keys;

        // Byte 5: Active LED Color Index (1:W, 2:B, 3:G, 4:R, 0:Off)
        // We look for which index in LED_Values is non-zero
        uint8_t active_color = 0;
        uint8_t active_brightness = 0;
        
        for(int i = 0; i < 4; i++) {
            if(LED_Values[i] > 0) {
                active_color = i + 1;       // Convert index to 1-4
                active_brightness = LED_Values[i];
                break; // Only one can be on, so we stop at the first one found
            }
        }

        tpdo_data[5] = active_color;      // Current Channel
        tpdo_data[6] = active_brightness; // Current Brightness (%)

        // Byte 7: Padding / Reserved
        tpdo_data[7] = 0x00; 

        // 5. Transmit at 100Hz (0x18B)
        CAN_Send_Msg(0x180 + MY_NODE_ID, tpdo_data, 8);

        // 6. 1ms loop delay
        Delay_ms(10);
    }
}
