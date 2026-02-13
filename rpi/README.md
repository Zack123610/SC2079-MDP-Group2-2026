# Raspberry Pi Main Program

Central communication hub that receives commands from Android via Bluetooth and forwards them to the STM32 board.

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

On Raspberry Pi, it is common to let a system service (or a simple startup script) create the RFCOMM serial device:

```bash
sudo rfcomm listen /dev/rfcomm0 1
```

This exposes the Bluetooth link as a serial port (`/dev/rfcomm0`), which `rpi/main.py` reads and writes like a normal UART.

## Usage

```bash
cd rpi
python main.py
```

By default:
- `main.py` connects to STM32 via USB (`stm32_interface.py`),
- then listens for Android messages on `/dev/rfcomm0` and forwards them.

The legacy PyBluez RFCOMM server (which binds and listens directly in Python) is still available behind a flag in `main.py` if you prefer that flow.

## Message Flow

```
Android App  --[Bluetooth RFCOMM]-->  Raspberry Pi  --[USB Serial]-->  STM32
                                      (/dev/rfcomm0)      (ttyACM0/ttyUSB0)
                                             main.py
```

1. Android connects to Pi via Bluetooth.
2. Android sends **high-level command strings**, for example:
   - Position message: `ROBOT|<y>,<x>,<DIRECTION>`
   - Movement command: `MOVE,<distance_cm>,<DIRECTION>`
   - Turn command:     `TURN,<LEFT|RIGHT>`
3. Raspberry Pi **parses** these messages and converts them into **4-byte binary command frames** for STM32:
   - `MOVE,30,FORWARD` -> frame `01 1E 00 00` (direction=1, distance=30 LE, pad)
   - `TURN,LEFT`       -> frame `03 00 00 00` (direction=3, distance=0, pad)
4. STM32 executes the motion and periodically sends back **4-byte status frames** reporting its angle and accumulated distance.
5. Once the accumulated distance reaches the commanded distance, Raspberry Pi automatically sends a **STOP** frame (`00 00 00 00`).

### 4-byte frame encoding (little-endian)

| Direction | Pi -> STM32 (command) | STM32 -> Pi (status) |
|-----------|-----------------------|----------------------|
| Byte 0    | Direction code (0-4)  | Angle (Z-axis)       |
| Byte 1-2  | Distance (uint16 cm)  | Accum distance (uint16 cm) |
| Byte 3    | Padding (0x00)        | Padding (0x00)       |

Direction codes: `0`=STOP, `1`=FORWARD, `2`=BACKWARD, `3`=LEFT, `4`=RIGHT

## Files

| File | Description |
|------|-------------|
| `main.py` | Main program – reads Android commands from `/dev/rfcomm0` (or legacy PyBluez server) and forwards to STM32 |
| `stm32_interface.py` | STM32 USB serial communication module (auto-detects `/dev/ttyACM*`) |
