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
                                           WiFi ────┤
                                                    │
                                          ┌─────────┴─────────┐
                                          │        PC         │
                                          │  Algo Service     │
                                          │  (:5001)          │
                                          │  YOLO Inference   │
                                          │  (ZMQ :5555/5556) │
                                          └───────────────────┘
```


## Components

| Component | Role |
|-----------|------|
| **Raspberry Pi** | Central communication hub. Bridges STM32, Android tablet, and PC. Hosts Pi Camera for vision. |
| **STM32 Board** | Controls car motors and movement based on commands from Pi. |
| **Android Tablet** | User interface for entering obstacle coordinates and navigation info. Connects to Pi via Bluetooth. |
| **PC (Laptop)** | Runs pathfinding algo service (:5001) and YOLO model inference for image recognition. |
| **Pi Camera** | Captures video stream sent to PC for real-time object detection. |

## Task 1 Flow

1. User enters obstacle positions on the Android tablet.
2. Android sends `ROBOT`, `OBSTACLE` messages, then `BEGIN` to RPi via Bluetooth.
3. RPi forwards obstacles to the **algo service** on PC (HTTP POST to `:5001/pathfinding`).
4. Algo service returns ordered path segments with movement instructions.
5. RPi translates instructions into STM32 commands, frames them, and batch-sends to STM32.
6. STM32 executes each 4-byte command sequentially, sending `DONE` after each.
7. RPi relays each completed instruction to Android as a UI update command.
8. On `CAPTURE_IMAGE` (`5000`), STM32 sends `HALT`; RPi runs image recognition via the YOLO detection stream, then sends `RESM` to resume.
9. Identified images are reported to Android as `TARGET,<obstacle>,<target_id>`.

## Android ↔ Raspberry Pi Protocol

**Transport**: Bluetooth RFCOMM (`/dev/rfcomm0`)

### Android → RPi

| Message | Format | Example |
|---------|--------|---------|
| Robot start position | `ROBOT,<x>,<y>,<direction>` | `ROBOT,0,0,NORTH` |
| Obstacle registration | `OBSTACLE,<id>,<x>,<y>,<direction>` | `OBSTACLE,1,120,120,NORTH` |
| Begin navigation | `BEGIN` | `BEGIN` |

Obstacle coordinates are in raw units (÷10 for grid position, e.g. 120 → grid 12).

### RPi → Android

| Message | Format | Example |
|---------|--------|---------|
| Forward/backward | `MOVE,<amount>,<FORWARD\|BACKWARD>` | `MOVE,50,FORWARD` |
| Arc turn | `TURN,<FORWARD_LEFT\|FORWARD_RIGHT\|BACKWARD_LEFT\|BACKWARD_RIGHT>` | `TURN,FORWARD_LEFT` |
| Stationary turn | `STAT_TURN,<LEFT\|RIGHT>` | `STAT_TURN,LEFT` |
| Image identified | `TARGET,<obstacle_id>,<target_id>` | `TARGET,3,12` |

## Raspberry Pi ↔ STM32 Protocol

**Transport**: USB Serial at 115200 baud (`/dev/ttyACM0`)

### Batch Command Frame

Instructions are concatenated and framed as: `<cmd1cmd2...cmdN>checksum`

Checksum = `(sum of all digit characters) % 100`

Example: `<603020206033>25`

### STM32 Command Codes (4 chars each)

| Code | Meaning |
|------|---------|
| `0000` | Stop |
| `1XXX` | Forward XXX cm (e.g. `1030` = 30 cm) |
| `2XXX` | Backward XXX cm |
| `3000` | Stationary turn left |
| `4000` | Stationary turn right |
| `5000` | Capture image (STM pauses, sends `HALT`, waits for `RESM`) |
| `6000` | Forward left |
| `7000` | Forward right |
| `8000` | Backward left |
| `9000` | Backward right |

### STM32 → RPi Responses

| Response | Meaning |
|----------|---------|
| `DONE` | Instruction completed, proceed to next |
| `HALT` | Paused at `5000`, waiting for RPi to finish image capture and send `RESM` |

## Image Recognition

- Video is streamed from Pi Camera to PC over ZMQ (port 5555)
- PC runs YOLO model to detect and identify numbers on obstacles
- Detection results are published back to RPi over ZMQ (port 5556)
- RPi uses a rolling detection window (majority vote) to confirm identifications

## Directory Structure

| Directory | Description |
|-----------|-------------|
| `rpi/` | Raspberry Pi modules – see [rpi/README.md](rpi/README.md) |
| `stm32/` | STM32 USB serial interface and test scripts |
| `service/` | Algo pathfinding REST API service (Flask, port 5001) |
| `imageReg/` | YOLO image recognition and PC-side receiver |

## Quick Start

```bash
# 1. Start algo service on PC
cd service
pipenv install && pipenv shell
python app.py

# 2. Start image recognition on PC
cd imageReg/src
python main.py

# 3. On Raspberry Pi
cd rpi
sudo rfcomm listen /dev/rfcomm0 1 &
python task1.py --pc-host <PC_IP>
```

See individual component READMEs for detailed setup instructions.
