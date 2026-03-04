# SC2079-MDP-Group2-2026

Project Repo for SC2079 Multi-Disciplinary Project 2026

## Project Overview

This project is a **mini car robot navigation challenge**. The robot must navigate to multiple obstacles and complete image recognition tasks by identifying numbers displayed on each obstacle.

## System Architecture

```
┌─────────────────┐   Bluetooth (RFCOMM)   ┌─────────────────┐
│  Android Tablet │◄──────────────────────►│                 │
│  (User Input)   │                        │  Raspberry Pi   │
└─────────────────┘                        │  (Comm Hub +    │
                                           │   Camera)       │
┌─────────────────┐          USB Serial    │                 │
│   STM32 Board   │◄──────────────────────►│                 │
│ (Motor Control) │                        │                 │
└─────────────────┘                        └────────┬────────┘
                                                   │
                                                   │ WiFi Video Stream
                                                   ▼
                                          ┌─────────────────┐
                                          │       PC        │
                                          │ (YOLO Inference │
                                          │  + Pathfinding) │
                                          └─────────────────┘
```


## Components

| Component | Role |
|-----------|------|
| **Raspberry Pi** | Central communication hub. Bridges STM32, Android tablet, and PC. Hosts Pi Camera for vision. |
| **STM32 Board** | Controls car motors and movement based on commands from Pi. |
| **Android Tablet** | User interface for entering obstacle coordinates and navigation info. Connects to Pi via Bluetooth. |
| **PC (Laptop)** | Runs YOLO model inference for image recognition and pathfinding algorithms. |
| **Pi Camera** | Captures video stream sent to PC for real-time object detection. |

## Image Recognition

- Video is streamed from Pi Camera to PC over WiFi
- PC runs YOLO model to detect and identify numbers on obstacles
- Results are used for navigation decisions

## STM32-RPI USB Interface

The `stm32/` folder contains the USB serial interface for communication between the Raspberry Pi and STM32F Board.

### Hardware Connection
- Connect STM32F Board's **UART2 port** (middle Micro USB port) to Raspberry Pi USB
- Communication runs at **115200 baudrate**
- Interface file: `/dev/ttyACM0` or `/dev/ttyUSB0` (auto-detected)

### Quick Start

```bash
cd stm32/src
pip install pyserial

# Test connection
python test_connection.py
```

### Usage

```python
from stm32_interface import STM32Interface

with STM32Interface() as stm:
    stm.send("FW050")  # Send command
    response = stm.receive()  # Get response
```

See `stm32/src/INSTRUCTION.md` for detailed setup instructions.

## Android ↔ Raspberry Pi Command Protocol (High-Level)

- **Transport**: Bluetooth RFCOMM.
- On Raspberry Pi this is typically exposed as a serial device created by a startup service, e.g.:

```bash
sudo rfcomm listen /dev/rfcomm0 1
```

- The Raspberry Pi program (`rpi/main.py`) can then read and write text lines over `/dev/rfcomm0`.

### Commands from Android to Raspberry Pi

- **Grid position request** (logical target in 2m×2m arena):
  - Format: `ROBOT|<y>,<x>,<DIRECTION>`
  - Example: `ROBOT|1,3,NORTH`
- **Movement command**:
  - Format: `MOVE,<distance_cm>,<DIRECTION>`
  - Example: `MOVE,30,FORWARD`
- **Turn command**:
  - Format: `TURN,<LEFT|RIGHT>`
  - Example: `TURN,LEFT`

The Raspberry Pi is responsible for interpreting these high-level commands and converting them into low-level motion instructions for the STM32.

## Raspberry Pi ↔ STM32 Movement Frames

All multi-byte values are **little-endian** (matching the STM32's native byte order).

- **Pi → STM32 (command frame, 4 bytes)**:
  - Byte 0: **Direction code** (uint8)  
    - `0` = STOP  
    - `1` = FORWARD  
    - `2` = BACKWARD  
    - `3` = LEFT  
    - `4` = RIGHT
  - Bytes 1–2: **Distance to travel** in centimetres (uint16 LE, 0–200 cm).
  - Byte 3: **Padding** (0x00).

- **STM32 → Pi (status frame, 4 bytes)**:
  - Byte 0: **Angle** (Z-axis rotation, uint8).
  - Bytes 1–2: **Accumulated distance travelled** in centimetres (uint16 LE).
  - Byte 3: **Padding** (0x00).

**Control logic (Pi side, planned):**
- When a movement command is issued, Raspberry Pi sends a 4-byte command frame to STM32.
- Raspberry Pi then reads 4-byte status frames from STM32 and tracks the accumulated distance.
- Once the accumulated distance reported by STM32 equals the commanded distance, Raspberry Pi sends a **STOP** command (direction code `0`) to halt the robot.
