"""
Raspberry Pi Main Program
Receives messages from Android and forwards to STM32.

Run on Raspberry Pi:
    python main.py

Dependencies:
    pip install pyserial pybluez

Bluetooth notes:
    - In many setups, the RFCOMM channel is already created by a
      system service such as:
          sudo rfcomm listen /dev/rfcomm0 1
    - In that case, this program reads/writes via /dev/rfcomm0
      instead of creating its own RFCOMM server socket.
    - The old PyBluez server code is kept for reference and can be
      re-enabled if needed.

Android message formats:
    ROBOT|<y>,<x>,<DIRECTION>       Position target on 2m x 2m grid
    MOVE,<distance_cm>,<DIRECTION>  Move forward/backward by distance
    TURN,<LEFT|RIGHT>               Turn in place

STM32 4-byte frame protocol:
    Pi -> STM32 (command):  [direction][dist_hi][dist_lo][pad]
    STM32 -> Pi (status):   [angle][accum_hi][accum_lo][pad]
"""

import struct
import sys
import time
from typing import Callable, Optional

import serial

from stm32_interface import STM32Interface

# Try importing bluetooth; it is only needed for the legacy server mode
# and may not be installed on every development machine.
try:
    import bluetooth  # type: ignore
except ImportError:
    bluetooth = None  # type: ignore


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# If True, use the original PyBluez RFCOMM server (bind/listen/accept).
# If False (recommended when using `rfcomm listen /dev/rfcomm0 1`),
# read/write commands via the /dev/rfcomm0 serial device instead.
USE_INTERNAL_BLUETOOTH_SERVER = False

RFCOMM_DEVICE = "/dev/rfcomm0"
RFCOMM_BAUDRATE = 115200

# Polling interval when waiting for STM32 status frames (seconds)
STM_POLL_INTERVAL = 0.05
# Maximum time to wait for a single movement to complete
STM_POLL_TIMEOUT = 30.0


# ---------------------------------------------------------------------------
# Direction codes (Pi -> STM32 command frame byte 0)
# ---------------------------------------------------------------------------

DIR_STOP = 0
DIR_FORWARD = 1
DIR_BACKWARD = 2
DIR_LEFT = 3
DIR_RIGHT = 4

DIRECTION_MAP: dict[str, int] = {
    "STOP": DIR_STOP,
    "FORWARD": DIR_FORWARD,
    "BACKWARD": DIR_BACKWARD,
    "LEFT": DIR_LEFT,
    "RIGHT": DIR_RIGHT,
}

# Reverse lookup for logging
DIR_NAME: dict[int, str] = {v: k for k, v in DIRECTION_MAP.items()}


# ---------------------------------------------------------------------------
# 4-byte frame helpers
# ---------------------------------------------------------------------------

def build_command_frame(direction: int, distance: int) -> str:
    """
    Build the command string to send to STM32.

    Format: "<direction><distance_3digits>" (ASCII).
    - 1 character: direction code '0'–'4' (STOP, FORWARD, BACKWARD, LEFT, RIGHT).
    - 3 characters: distance in cm, zero-padded (e.g. 030 for 30 cm).

    Examples:
        MOVE,30,FORWARD  -> "1030"  (1=forward, 030=30 cm)
        TURN,LEFT        -> "3000"  (3=left, 000=0 cm)
        STOP             -> "0000"

    Args:
        direction: Direction code (0-4).
        distance:  Distance in centimetres (0-200).

    Returns:
        4-character string, e.g. "1030".
    """
    direction = max(0, min(direction, 4))
    distance = max(0, min(distance, 200))
    return f"{direction}{distance:03d}"


# ---------------------------------------------------------------------------
# STM32 status parsing (currently unused – Pi does not receive from STM)
# ---------------------------------------------------------------------------
# def parse_status_frame(data: bytes) -> tuple[int, int]:
#     """
#     Parse a 4-byte status frame received from STM32.
#
#     Frame layout (4 bytes, little-endian):
#         Byte 0 : Angle (Z-axis rotation, uint8)
#         Byte 1 : Accumulated distance low    }
#         Byte 2 : Accumulated distance high   }  uint16, cm
#         Byte 3 : Padding
#
#     Returns:
#         (angle, accumulated_distance_cm)
#     """
#     angle, accum_dist, _pad = struct.unpack("<BHB", data)
#     return angle, accum_dist


# ---------------------------------------------------------------------------
# Android message parser
# ---------------------------------------------------------------------------

def parse_android_message(message: str) -> Optional[tuple[int, int]]:
    """
    Parse a high-level command string from the Android tablet.

    Supported formats
    -----------------
    ROBOT|<y>,<x>,<DIRECTION>
        Grid-position target on the 2 m x 2 m arena.
        *Not yet converted to movement commands* (requires pathfinding).
        Returns ``None`` for now.

    MOVE,<distance_cm>,<DIRECTION>
        Direct movement.  ``DIRECTION`` is FORWARD or BACKWARD.
        Returns ``(direction_code, distance_cm)``.

    TURN,<LEFT|RIGHT>
        In-place turn.  Distance is 0 (STM32 handles the turn amount).
        Returns ``(direction_code, 0)``.

    Returns:
        ``(direction_code, distance_cm)`` ready for
        :func:`build_command_frame`, or ``None`` if the message cannot
        be turned into a single movement command.
    """
    msg = message.strip()

    # ----- Position message: ROBOT|y,x,DIRECTION -----
    if msg.startswith("ROBOT|"):
        payload = msg.split("|", 1)[1]  # "y,x,DIRECTION"
        parts = [p.strip() for p in payload.split(",")]
        if len(parts) == 3:
            try:
                y = int(parts[0])
                x = int(parts[1])
                direction_str = parts[2].upper()
            except ValueError:
                print(f"[PARSE] Invalid ROBOT coords: {message}")
                return None
            print(f"[PARSE] Position target: y={y}, x={x}, facing={direction_str}")
            # TODO: Convert grid target (x, y, direction) into a series of
            #       movement commands via pathfinding / planning.
            print("[PARSE]   (position-to-movement conversion not yet implemented)")
        else:
            print(f"[PARSE] Malformed ROBOT message: {message}")
        return None

    # ----- Movement command: MOVE,<distance>,<DIRECTION> -----
    if msg.upper().startswith("MOVE,"):
        parts = [p.strip() for p in msg.split(",")]
        if len(parts) == 3:
            try:
                distance = int(parts[1])
            except ValueError:
                print(f"[PARSE] Invalid MOVE distance: {message}")
                return None
            dir_str = parts[2].upper()
            dir_code = DIRECTION_MAP.get(dir_str)
            if dir_code is None:
                print(f"[PARSE] Unknown direction '{dir_str}' in: {message}")
                return None
            print(f"[PARSE] MOVE {distance} cm {dir_str}")
            return (dir_code, distance)
        print(f"[PARSE] Malformed MOVE message: {message}")
        return None

    # ----- Turn command: TURN,<LEFT|RIGHT> -----
    if msg.upper().startswith("TURN,"):
        parts = [p.strip() for p in msg.split(",")]
        if len(parts) == 2:
            dir_str = parts[1].upper()
            dir_code = DIRECTION_MAP.get(dir_str)
            if dir_code is None:
                print(f"[PARSE] Unknown turn direction '{dir_str}' in: {message}")
                return None
            # For turns, distance = 0; the STM32 decides the rotation amount
            print(f"[PARSE] TURN {dir_str}")
            return (dir_code, 0)
        print(f"[PARSE] Malformed TURN message: {message}")
        return None

    print(f"[PARSE] Unrecognised message: {message}")
    return None


# ---------------------------------------------------------------------------
# Movement execution: Pi only forwards command to STM32 (no receive from STM)
# ---------------------------------------------------------------------------

def execute_movement(
    stm: STM32Interface,
    direction: int,
    distance: int,
    bt_reply_fn: Optional[Callable[[str], None]] = None,
) -> bool:
    """
    Parse Android command into STM32 format and send to STM32.
    Raspberry Pi does not receive or process any message from STM board.
    """
    frame = build_command_frame(direction, distance)
    dir_name = DIR_NAME.get(direction, str(direction))
    print(f"[CMD] Sending: direction={dir_name}, distance={distance} cm, "
          f"frame=\"{frame}\"")

    if not stm.send(frame, add_newline=False):
        print("[CMD] Failed to send command to STM32")
        if bt_reply_fn:
            bt_reply_fn("SEND_FAIL")
        return False

    if bt_reply_fn:
        bt_reply_fn("OK")
    return True


# ---------------------------------------------------------------------------
# Handle a single Android message  (shared by both BT modes)
# ---------------------------------------------------------------------------

def handle_android_message(
    message: str,
    stm: STM32Interface,
    bt_reply_fn: Optional[Callable[[str], None]] = None,
) -> None:
    """
    Parse *message* from Android, execute the resulting movement on STM32,
    and optionally send a reply back to Android via *bt_reply_fn*.
    """
    result = parse_android_message(message)
    if result is None:
        # Not a movement command (e.g. ROBOT| position or unknown)
        if bt_reply_fn:
            bt_reply_fn("UNKNOWN_CMD")
        return

    direction, distance = result
    execute_movement(stm, direction, distance, bt_reply_fn=bt_reply_fn)


# ---------------------------------------------------------------------------
# Legacy PyBluez server  (kept for reference / fallback)
# ---------------------------------------------------------------------------

def setup_bluetooth(channel: int = 1):
    """
    Setup Bluetooth RFCOMM server socket (legacy mode).

    Normally you do NOT need this if a system service already runs:
        sudo rfcomm listen /dev/rfcomm0 1
    """
    if bluetooth is None:
        print("[BT] pybluez is not installed – cannot use legacy server.")
        sys.exit(1)

    server = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    server.bind(("", channel))
    server.listen(1)

    # Make discoverable
    bluetooth.advertise_service(
        server,
        "MDP-RPI",
        service_classes=[bluetooth.SERIAL_PORT_CLASS],
        profiles=[bluetooth.SERIAL_PORT_PROFILE],
    )

    print(f"[BT] Waiting for Android connection on channel {channel}...")
    return server


def run_legacy_bluetooth_server(stm: STM32Interface) -> None:
    """Run the original PyBluez Bluetooth server loop."""
    bt_server = setup_bluetooth()

    try:
        while True:
            # Accept Android connection
            client, addr = bt_server.accept()
            print(f"[BT] Android connected: {addr}")

            def _bt_reply(text: str) -> None:
                try:
                    client.send((text + "\n").encode("utf-8"))
                    print(f"[BT] Sent to Android: {text}")
                except Exception as e:
                    print(f"[BT] Reply error: {e}")

            try:
                while True:
                    # Receive message from Android
                    data = client.recv(1024)
                    if not data:
                        break

                    message = data.decode("utf-8").strip()
                    print(f"[BT] Received: {message}")
                    handle_android_message(message, stm, bt_reply_fn=_bt_reply)

            except Exception as e:
                print(f"[BT] Connection error: {e}")
            finally:
                client.close()
                print("[BT] Android disconnected")

    except KeyboardInterrupt:
        print("\n[INFO] Shutting down (legacy Bluetooth server)...")
    finally:
        bt_server.close()


# ---------------------------------------------------------------------------
# RFCOMM serial mode  (preferred with `rfcomm listen /dev/rfcomm0 1`)
# ---------------------------------------------------------------------------

def run_rfcomm_serial(stm: STM32Interface) -> None:
    """
    Listen for Android commands on /dev/rfcomm0 and forward to STM32.

    Assumes a system service (or manual command) has already bound the
    RFCOMM channel::

        sudo rfcomm listen /dev/rfcomm0 1
    """
    try:
        ser = serial.Serial(
            RFCOMM_DEVICE,
            RFCOMM_BAUDRATE,
            timeout=1.0,
        )
    except serial.SerialException as e:
        print(f"[BT] Failed to open {RFCOMM_DEVICE}: {e}")
        print("[BT] Falling back to idle loop. Check rfcomm setup.")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\n[INFO] Exiting.")
        return

    print(f"[BT] Listening for Android on {RFCOMM_DEVICE} "
          f"(baud={RFCOMM_BAUDRATE})...")

    def _bt_reply(text: str) -> None:
        try:
            ser.write((text + "\n").encode("utf-8"))
            ser.flush()
            print(f"[BT] Sent to Android: {text}")
        except Exception as e:
            print(f"[BT] Reply error: {e}")

    try:
        while True:
            line = ser.readline()
            if not line:
                continue

            message = line.decode("utf-8", errors="ignore").strip()
            if not message:
                continue

            print(f"[BT] Received: {message}")
            handle_android_message(message, stm, bt_reply_fn=_bt_reply)

    except KeyboardInterrupt:
        print("\n[INFO] Shutting down (RFCOMM serial mode)...")
    finally:
        ser.close()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    # Connect to STM32
    print("[STM32] Connecting...")
    stm = STM32Interface()
    if not stm.is_connected:
        print("[STM32] Failed to connect. Exiting.")
        sys.exit(1)
    print("[STM32] Connected")

    try:
        if USE_INTERNAL_BLUETOOTH_SERVER:
            print("[MODE] Using internal PyBluez RFCOMM server.")
            run_legacy_bluetooth_server(stm)
        else:
            print("[MODE] Using /dev/rfcomm0 serial (preferred).")
            run_rfcomm_serial(stm)
    finally:
        stm.close()


if __name__ == "__main__":
    main()
