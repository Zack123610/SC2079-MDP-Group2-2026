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
3. Raspberry Pi parses these messages (to be implemented) and converts them into **low-level movement frames** for STM32.
4. STM32 executes the motion and periodically reports its pose (angle + accumulated distance) back to the Pi in 4-byte status frames (planned).
5. Once the accumulated distance reaches the commanded distance, Raspberry Pi will issue a **STOP** command to STM32.

> Note: The exact 4-byte encoding between Raspberry Pi and STM32 is still being finalised, but the intended semantics are:
>
> - **Pi → STM32 (command frame, 4 bytes)**  
>   `[direction][distance_byte1][distance_byte2][padding]`
> - **STM32 → Pi (status frame, 4 bytes)**  
>   `[angle][distance_byte1][distance_byte2][padding]`

## Files

| File | Description |
|------|-------------|
| `main.py` | Main program – reads Android commands from `/dev/rfcomm0` (or legacy PyBluez server) and forwards to STM32 |
| `stm32_interface.py` | STM32 USB serial communication module (auto-detects `/dev/ttyACM*`) |
