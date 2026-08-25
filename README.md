# CAN Communication Protocol Specification - Node ID 11

This document outlines the CAN communication interface for the STM32-based controller, covering real-time telemetry, direct hardware control, and on-demand data access.

## 1. System Configuration
* **Baud Rate:** 1 Mbps
* **Node ID:** 11 (0x0B) for Right Joystick ADC
* **Node ID:** 15 (0x0F) for Left Joystick ADC
* **Refresh Rate:** 100 Hz (10ms interval)
* **CAN Mode:** Normal Mode
* **CAN Hardware:** PA11 (RX), PA12 (TX)

## 2. Identifier (COB-ID) Mapping
Based on the Node ID **11 (0x0B)**, the system utilizes the following identifiers:

| Function | Direction | COB-ID (Hex) | Description |
| :--- | :--- | :--- | :--- |
| **TPDO1** | Device → Bus | `0x18B` | Periodic Sensor, Key, and LED Feedback (0x180 + NodeID) |
| **RPDO1** | Bus → Device | `0x20B` | Direct LED Control (0x200 + NodeID) |
| **TSDO** | Device → Bus | `0x58B` | SDO Upload Response (0x580 + NodeID) |
| **RSDO** | Bus → Device | `0x60B` | SDO Upload Request (0x600 + NodeID) |

## 3. Real-Time Telemetry (TPDO1)
The device broadcasts a Transmit Process Data Object (TPDO) every **10ms** containing the current state of sensors, buttons, and the active LED status.

**CAN ID:** `0x18B` | **Data Length:** 8 Bytes

| Byte | Data Field | Type | Description |
| :--- | :--- | :--- | :--- |
| 0 | Hall 1 LSB | `uint8` | MCP3202 CH0 Lower 8 bits |
| 1 | Hall 1 MSB | `uint4` | MCP3202 CH0 Upper 4 bits (0x0F mask) |
| 2 | Hall 2 LSB | `uint8` | MCP3202 CH1 Lower 8 bits |
| 3 | Hall 2 MSB | `uint4` | MCP3202 CH1 Upper 4 bits (0x0F mask) |
| 4 | Key State | `uint8` | Bitmask: Bit0:PB0, Bit1:PB1, Bit2:PA1, Bit3:PA2 |
| 5 | **Active Color**| `uint8` | **1:White, 2:Blue, 3:Green, 4:Red (0: Off)** |
| 6 | **Brightness** | `uint8` | **Current PWM Intensity (0 - 100%)** |
| 7 | Reserved | `uint8` | Fixed 0x00 |

## 4. Direct LED Control (RPDO1)
Control the active LED and its brightness. Note: Hardware logic ensures only one LED is active at a time; setting a new color index disables others.

**CAN ID:** `0x20B` | **Data Length:** 2 Bytes

| Byte | Parameter | Range | Description |
| :--- | :--- | :--- | :--- |
| 0 | Channel | 1 - 4 | 1:White, 2:Blue, 3:Green, 4:Red |
| 1 | Brightness | 0 - 100 | PWM Duty Cycle (%) |

## 5. Object Dictionary (SDO)
The device supports on-demand data retrieval for all internal parameters.

### Access Table
| Index (Hex) | Sub-Index | Description | Data Type |
| :--- | :--- | :--- | :--- |
| `0x2000` | 1 | Hall Sensor 1 (Raw ADC) | `uint16` |
| `0x2000` | 2 | Hall Sensor 2 (Raw ADC) | `uint16` |
| `0x2001` | 1 - 4 | LED Brightness (W, B, G, R) | `uint8` |
| `0x2002` | 0 | Key State Bitmask | `uint8` |

## 6. Peripheral Pinout Summary
* **CAN:** PA11 (RX), PA12 (TX)
* **ADC (MCP3202):** PA4 (CS), PA5 (SCK), PA6 (MISO), PA7 (MOSI)
* **LED (PWM):** PB6 (White), PB7 (Blue), PB8 (Green), PB9 (Red)
* **Keys:** PA1, PA2, PB0, PB1 (Internal Pull-Up)

---
*Generated for Node 11 Joystick Controller Firmware v1.1*
