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

# Make Pi discoverable
sudo hciconfig hci0 piscan
```

### 3. Connect STM32

Connect STM32F Board's UART2 (middle Micro USB port) to Raspberry Pi USB.

## Usage

```bash
python main.py
```

## Flow

```
Android App  --[Bluetooth]-->  Raspberry Pi  --[USB Serial]-->  STM32
                               (main.py)
```

1. Android connects to Pi via Bluetooth
2. Android sends command (e.g., "FW050")
3. Pi forwards command to STM32
4. STM32 responds
5. Pi sends response back to Android

## Files

| File | Description |
|------|-------------|
| `main.py` | Main program - Bluetooth server + STM32 forwarding |
| `stm32_interface.py` | STM32 serial communication module |
