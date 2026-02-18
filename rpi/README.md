# Raspberry Pi Main Program

Central communication hub that receives commands from Android via Bluetooth and forwards them to the STM32 board.

## Module Structure

```
rpi/
├── main.py                 # Entry point – routing logic only
├── bluetooth_interface.py  # Bluetooth RFCOMM link to Android tablet
└── stm32_interface.py      # USB serial link to STM32 board
```

| Module | Responsibility |
|--------|---------------|
| `main.py` | Starts both interfaces, parses Android messages, routes commands to STM32 |
| `bluetooth_interface.py` | Opens /dev/rfcomm0 (or PyBluez server), exposes `readline()` / `send()` |
| `stm32_interface.py` | Opens /dev/ttyACM0 (auto-detected), exposes `send()` / `receive()` |

## Setup

### 1. Install Dependencies

```bash
# System packages for Bluetooth
sudo apt install bluetooth bluez libbluetooth-dev

# Python packages
pip install pyserial pybluez
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

```bash
cd rpi
python main.py
```

By default `main.py`:
1. Connects to the STM32 via USB (`stm32_interface.py`).
2. Opens `/dev/rfcomm0` for Android communication (`bluetooth_interface.py`).
3. Loops: reads one line from Android → parses → sends command to STM32.

To use the legacy PyBluez RFCOMM server instead (Python creates the socket itself), set in `main.py`:

```python
USE_PYBLUEZ_SERVER = True
```

## Message Flow

```
Android App  --[Bluetooth RFCOMM]-->  Raspberry Pi  --[USB Serial]-->  STM32
             bluetooth_interface.py       main.py       stm32_interface.py
             (/dev/rfcomm0)                             (/dev/ttyACM0)
```

1. Android sends a **high-level command string** (newline-terminated):
   - Position message: `ROBOT|<y>,<x>,<DIRECTION>`
   - Movement command: `MOVE,<distance_cm>,<DIRECTION>`
   - Turn command:     `TURN,<LEFT|RIGHT>`

2. `main.py` parses the message and builds a **4-character ASCII command**:

   | Android message   | STM32 command | Meaning              |
   |-------------------|---------------|----------------------|
   | `MOVE,30,FORWARD` | `1030`        | Forward 30 cm        |
   | `MOVE,50,BACKWARD`| `2050`        | Backward 50 cm       |
   | `TURN,LEFT`       | `3000`        | Turn left            |
   | `TURN,RIGHT`      | `4000`        | Turn right           |
   | *(STOP)*          | `0000`        | Stop                 |

   Format: `<direction_code><distance_3digits>`
   - `0` = STOP, `1` = FORWARD, `2` = BACKWARD, `3` = LEFT, `4` = RIGHT

3. `stm32_interface.py` sends the command string over USB serial.
4. Raspberry Pi replies `OK` (or `SEND_FAIL`) to Android.

## Testing Individual Modules

```bash
# Test Bluetooth interface only (echo server)
python bluetooth_interface.py

# Test STM32 interface only (interactive send/receive)
python stm32_interface.py
```
