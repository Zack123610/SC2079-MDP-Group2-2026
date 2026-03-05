# Raspberry Pi Main Program

Central communication hub that coordinates Android (Bluetooth), STM32 (USB serial), PC algo service (HTTP), and PC image recognition (ZMQ).

## Module Structure

```
rpi/
├── task1.py                # Task 1 entry point – full autonomous navigation flow
├── obstacle_a5.py          # Obstacle A5 – standalone image identification task
├── main.py                 # Legacy entry point – basic Android → STM32 routing
├── bluetooth_interface.py  # Bluetooth RFCOMM link to Android tablet
├── stm32_interface.py      # USB serial link to STM32 board
├── algo_interface.py       # HTTP client for algo pathfinding service on PC
├── pc_interface.py         # ZMQ SUB listener for YOLO detection results from PC
├── camera_interface.py     # ZMQ PUB streamer for Pi Camera → PC
└── convert_instructions.py # Standalone tool to convert algo JSON → STM/Android commands
```

| Module | Responsibility |
|--------|---------------|
| `task1.py` | Full Task 1 flow: collects obstacles from Android, requests path from algo service, batch-sends instructions to STM32, handles image capture, reports results to Android |
| `obstacle_a5.py` | Interactive obstacle image identification with rolling detection window and face rotation |
| `main.py` | Legacy: starts Bluetooth + STM32 interfaces, routes Android commands to STM32 |
| `bluetooth_interface.py` | Opens `/dev/rfcomm0` (or PyBluez server), exposes `readline()` / `send()` |
| `stm32_interface.py` | Opens `/dev/ttyACM0` (auto-detected), exposes `send()` / `receive()` |
| `algo_interface.py` | HTTP client for the pathfinding REST API on the PC (default port 5001) |
| `pc_interface.py` | ZMQ SUB client for YOLO detection results published by PC |
| `camera_interface.py` | ZMQ PUB streamer – captures Pi Camera frames and publishes to PC |
| `convert_instructions.py` | Standalone CLI tool to convert algo service JSON responses into STM32 commands and Android UI commands |

## Setup

### 1. Install Dependencies

```bash
# System packages for Bluetooth
sudo apt install bluetooth bluez libbluetooth-dev

# Python packages
pip install pyserial pybluez pyzmq requests opencv-python
```

### 2. Enable Bluetooth

```bash
sudo systemctl enable bluetooth
sudo systemctl start bluetooth

# Make Pi discoverable (if using classic pairing flow)
sudo hciconfig hci0 piscan
```

### 3. Bind RFCOMM Channel (Recommended)

Let a system service (or startup script) create the RFCOMM serial device before running the program:

```bash
sudo rfcomm listen /dev/rfcomm0 1
```

This exposes the Bluetooth link as `/dev/rfcomm0`, which `bluetooth_interface.py` reads and writes like a normal UART.

## Usage

### Task 1 – Obstacle Navigation & Image Recognition

```bash
cd rpi
python task1.py --pc-host <PC_IP>
```

`task1.py` connects all interfaces and runs the full autonomous flow:

1. Waits for obstacle info from Android via Bluetooth:
   ```
   ROBOT,0,0,NORTH
   OBSTACLE,1,120,120,NORTH
   OBSTACLE,2,90,80,SOUTH
   BEGIN
   ```
2. Requests a pathfinding solution from the algo service on PC.
3. Batch-sends all STM32 instructions in one framed payload (`<cmds>checksum`).
4. Processes STM32 responses one-by-one:
   - `DONE` → sends the corresponding Android UI command.
   - `HALT` (from `5000` / CAPTURE_IMAGE) → runs image recognition via DetectionTracker, sends `TARGET,<obs>,<id>` to Android, then sends `RESM` to resume STM.
5. After completion (or 6-minute timeout), sends `0000` to STM and waits for next run.

### Convert Instructions (standalone tool)

```bash
# From a JSON file
python convert_instructions.py --file response.json

# Interactive (paste JSON then Ctrl-D)
python convert_instructions.py
```

Converts an algo service JSON response into a table of STM32 commands and Android UI commands, plus the framed STM payload string.

### Testing Individual Modules

```bash
# Test Bluetooth interface only (bidirectional echo)
python bluetooth_interface.py

# Test STM32 interface only (interactive send/receive)
python stm32_interface.py

# Test algo service connection
python algo_interface.py --host <PC_IP> --port 5001

# Test camera streaming
python camera_interface.py

# Test PC detection listener
python pc_interface.py --host <PC_IP>
```

## Message Flow (Task 1)

```
Android App  --[Bluetooth RFCOMM]-->  Raspberry Pi  --[USB Serial]-->  STM32
                                           │
                                           ├──[HTTP POST]──> PC Algo Service (:5001)
                                           ├──[ZMQ PUB]───> PC (camera frames)
                                           └──[ZMQ SUB]<─── PC (YOLO detections)
```

### Android → RPi

| Message | Example | Description |
|---------|---------|-------------|
| Robot position | `ROBOT,0,0,NORTH` | Starting position and direction |
| Obstacle info | `OBSTACLE,1,120,120,NORTH` | Obstacle 1 at grid (12,12) facing North |
| Start execution | `BEGIN` | Trigger pathfinding and navigation |

### RPi → Android

| Message | Example | Description |
|---------|---------|-------------|
| Movement | `MOVE,50,FORWARD` | Robot moved forward 50 cm |
| Turn | `TURN,FORWARD_LEFT` | Robot turned forward-left |
| Stationary turn | `STAT_TURN,LEFT` | Robot turned left in place |
| Image result | `TARGET,3,12` | Obstacle 3 identified as target 12 |

### RPi → STM32

Instructions are sent as a single framed string: `<cmd1cmd2...cmdN>checksum`

| STM32 command | Meaning |
|---------------|---------|
| `1030` | Forward 30 cm |
| `2030` | Backward 30 cm |
| `3000` | Stationary turn left |
| `4000` | Stationary turn right |
| `5000` | Capture image (STM sends HALT, waits for RESM) |
| `6000` | Forward left |
| `7000` | Forward right |
| `8000` | Backward left |
| `9000` | Backward right |
| `0000` | Stop |
