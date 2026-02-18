"""
Raspberry Pi Main Program

Bridges the Android tablet (Bluetooth) and the STM32 motor controller (USB serial).

Run on Raspberry Pi:
    python main.py

Dependencies:
    pip install pyserial pybluez

Modules:
    bluetooth_interface.py  -- Bluetooth RFCOMM link to Android tablet
    stm32_interface.py      -- USB serial link to STM32 board

Android message formats:
    ROBOT|<y>,<x>,<DIRECTION>       Position target on 2m x 2m grid (pathfinding TBD)
    MOVE,<distance_cm>,<DIRECTION>  Move forward/backward by distance
    TURN,<LEFT|RIGHT>               Turn in place

STM32 command format (ASCII, 4 chars):
    "<direction><distance_3digits>"
    e.g. "1030" = FORWARD 30 cm,  "3000" = TURN LEFT,  "0000" = STOP
"""

import sys
import time
from typing import Optional

from bluetooth_interface import BluetoothInterface
from stm32_interface import STM32Interface


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# Set to True to use the legacy PyBluez RFCOMM server instead of /dev/rfcomm0.
USE_PYBLUEZ_SERVER = False


# ---------------------------------------------------------------------------
# Direction codes  (Pi -> STM32 command)
# ---------------------------------------------------------------------------

DIR_STOP     = 0
DIR_FORWARD  = 1
DIR_BACKWARD = 2
DIR_LEFT     = 3
DIR_RIGHT    = 4

DIRECTION_MAP: dict[str, int] = {
    "STOP":     DIR_STOP,
    "FORWARD":  DIR_FORWARD,
    "BACKWARD": DIR_BACKWARD,
    "LEFT":     DIR_LEFT,
    "RIGHT":    DIR_RIGHT,
}

DIR_NAME: dict[int, str] = {v: k for k, v in DIRECTION_MAP.items()}


# ---------------------------------------------------------------------------
# STM32 command helpers
# ---------------------------------------------------------------------------

def build_command(direction: int, distance: int) -> str:
    """
    Build the 4-character ASCII command string for the STM32.

    Format: "<direction><distance_3digits>"
      - 1 char : direction code '0'–'4'
      - 3 chars: distance in cm, zero-padded (000–200)

    Examples:
        MOVE,30,FORWARD  -> "1030"
        TURN,LEFT        -> "3000"
        STOP             -> "0000"
    """
    direction = max(0, min(direction, 4))
    distance  = max(0, min(distance, 200))
    return f"{direction}{distance:03d}"


# ---------------------------------------------------------------------------
# Android message parser
# ---------------------------------------------------------------------------

def parse_android_message(message: str) -> Optional[tuple[int, int]]:
    """
    Parse a high-level command string from Android.

    Returns:
        ``(direction_code, distance_cm)`` or ``None`` if the message is not
        a recognised movement command.
    """
    msg = message.strip()

    # ROBOT|<y>,<x>,<DIRECTION>  -- grid position target
    if msg.startswith("ROBOT|"):
        payload = msg.split("|", 1)[1]
        parts = [p.strip() for p in payload.split(",")]
        if len(parts) == 3:
            try:
                y, x = int(parts[0]), int(parts[1])
                direction_str = parts[2].upper()
            except ValueError:
                print(f"[PARSE] Invalid ROBOT coords: {message}")
                return None
            print(f"[PARSE] Position target: y={y}, x={x}, facing={direction_str}")
            # TODO: Convert grid target into movement sequence via pathfinding.
            print("[PARSE]   (position-to-movement conversion not yet implemented)")
        else:
            print(f"[PARSE] Malformed ROBOT message: {message}")
        return None

    # MOVE,<distance>,<DIRECTION>
    if msg.upper().startswith("MOVE,"):
        parts = [p.strip() for p in msg.split(",")]
        if len(parts) == 3:
            try:
                distance = int(parts[1])
            except ValueError:
                print(f"[PARSE] Invalid MOVE distance: {message}")
                return None
            dir_code = DIRECTION_MAP.get(parts[2].upper())
            if dir_code is None:
                print(f"[PARSE] Unknown direction '{parts[2]}' in: {message}")
                return None
            print(f"[PARSE] MOVE {distance} cm {parts[2].upper()}")
            return (dir_code, distance)
        print(f"[PARSE] Malformed MOVE message: {message}")
        return None

    # TURN,<LEFT|RIGHT>
    if msg.upper().startswith("TURN,"):
        parts = [p.strip() for p in msg.split(",")]
        if len(parts) == 2:
            dir_code = DIRECTION_MAP.get(parts[1].upper())
            if dir_code is None:
                print(f"[PARSE] Unknown turn direction '{parts[1]}' in: {message}")
                return None
            print(f"[PARSE] TURN {parts[1].upper()}")
            return (dir_code, 0)
        print(f"[PARSE] Malformed TURN message: {message}")
        return None

    print(f"[PARSE] Unrecognised message: {message}")
    return None


# ---------------------------------------------------------------------------
# Message routing  (Android -> STM32)
# ---------------------------------------------------------------------------

def handle_message(
    message: str,
    stm: STM32Interface,
    bt: BluetoothInterface,
) -> None:
    """
    Parse one Android message, convert it to a STM32 command, and send it.
    Reply "OK" or an error string back to Android.
    """
    result = parse_android_message(message)
    if result is None:
        bt.send("UNKNOWN_CMD")
        return

    direction, distance = result
    cmd = build_command(direction, distance)
    dir_name = DIR_NAME.get(direction, str(direction))
    print(f"[CMD] Sending to STM32: direction={dir_name}, "
          f"distance={distance} cm, cmd=\"{cmd}\"")

    if stm.send(cmd, add_newline=False):
        bt.send("OK")
    else:
        print("[CMD] Failed to send command to STM32")
        bt.send("SEND_FAIL")


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main() -> None:
    # --- Connect to STM32 ---------------------------------------------------
    print("[STM32] Connecting...")
    stm = STM32Interface()
    if not stm.is_connected:
        print("[STM32] Failed to connect. Exiting.")
        sys.exit(1)
    print("[STM32] Connected")

    # --- Open Bluetooth link ------------------------------------------------
    bt = BluetoothInterface(use_pybluez_server=USE_PYBLUEZ_SERVER)
    if not bt.start():
        print("[BT] Failed to open Bluetooth. Exiting.")
        stm.close()
        sys.exit(1)

    print("[INFO] Ready. Waiting for Android commands...")

    try:
        while True:
            message = bt.readline()
            if not message:
                time.sleep(0.01)
                continue

            print(f"[BT] Received: {message}")
            handle_message(message, stm, bt)

    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")
    finally:
        bt.close()
        stm.close()


if __name__ == "__main__":
    main()
