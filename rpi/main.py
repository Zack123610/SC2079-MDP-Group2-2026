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
    - In that case, this program should read/write via /dev/rfcomm0
      instead of creating its own RFCOMM server socket.
    - The old PyBluez server code is kept for reference and can be
      re-enabled if needed.
"""

import sys
import time

import bluetooth  # kept for optional legacy server mode
import serial

from stm32_interface import STM32Interface


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# If True, use the original PyBluez RFCOMM server (bind/listen/accept).
# If False (recommended when using `rfcomm listen /dev/rfcomm0 1`),
# read/write commands via the /dev/rfcomm0 serial device instead.
USE_INTERNAL_BLUETOOTH_SERVER = False

RFCOMM_DEVICE = "/dev/rfcomm0"
RFCOMM_BAUDRATE = 115200


# ---------------------------------------------------------------------------
# Legacy PyBluez server (kept for reference / fallback)
# ---------------------------------------------------------------------------

def setup_bluetooth(channel: int = 1) -> bluetooth.BluetoothSocket:
    """
    Setup Bluetooth RFCOMM server socket (legacy mode).

    Normally you do NOT need this if a system service already runs:
        sudo rfcomm listen /dev/rfcomm0 1
    """
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

            try:
                while True:
                    # Receive message from Android
                    data = client.recv(1024)
                    if not data:
                        break

                    message = data.decode("utf-8").strip()
                    print(f"[BT] Received: {message}")

                    # TODO: Parse Android high-level commands here, e.g.:
                    #  - ROBOT|<y>,<x>,<DIRECTION>
                    #  - MOVE,<distance_cm>,<DIRECTION>
                    #  - TURN,<LEFT|RIGHT>
                    #
                    # For now we simply forward the raw string to STM32.

                    # Forward to STM32
                    stm.send(message)

                    # Get STM32 response and send back to Android
                    response = stm.receive(timeout=2.0)
                    if response:
                        client.send(response.encode("utf-8"))
                        print(f"[BT] Sent to Android: {response}")

            except bluetooth.BluetoothError as e:
                print(f"[BT] Connection error: {e}")
            finally:
                client.close()
                print("[BT] Android disconnected")

    except KeyboardInterrupt:
        print("\n[INFO] Shutting down (legacy Bluetooth server)...")
    finally:
        bt_server.close()


# ---------------------------------------------------------------------------
# RFCOMM serial mode (preferred with `rfcomm listen /dev/rfcomm0 1`)
# ---------------------------------------------------------------------------

def run_rfcomm_serial(stm: STM32Interface) -> None:
    """
    Listen for Android commands on /dev/rfcomm0 and forward to STM32.

    This assumes a system service (or manual command) has already bound
    the RFCOMM channel, e.g.:
        sudo rfcomm listen /dev/rfcomm0 1

    Data is expected as UTF-8 text lines terminated by '\\n'.
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
        # Idle loop so program does not immediately exit
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\n[INFO] Exiting.")
        return

    print(f"[BT] Listening for Android on {RFCOMM_DEVICE} (baud={RFCOMM_BAUDRATE})...")

    try:
        while True:
            line = ser.readline()
            if not line:
                continue

            message = line.decode("utf-8", errors="ignore").strip()
            if not message:
                continue

            print(f"[BT] Received: {message}")

            # TODO: Parse high-level commands from Android here:
            #   - Position messages:  ROBOT|<y>,<x>,<DIRECTION>
            #   - Movement commands: MOVE,<distance_cm>,<DIRECTION>
            #   - Turn commands:     TURN,<LEFT|RIGHT>
            #
            # The Raspberry Pi will eventually:
            #   1) Convert these to a 4-byte movement frame for STM32:
            #        [dir_byte][distance_byte1][distance_byte2][pad]
            #   2) Track STM32 status frames of the same 4-byte format:
            #        [angle_byte][distance_byte1][distance_byte2][pad]
            #   3) When accumulated distance == target distance,
            #      send a STOP command (direction=0).
            #
            # That low-level 4-byte protocol is still being finalised,
            # so for now we just forward the raw text to STM32.

            stm.send(message)

            # Optionally read a single response back from STM32
            response = stm.receive(timeout=2.0)
            if response:
                ser.write((response + "\n").encode("utf-8"))
                ser.flush()
                print(f"[BT] Sent to Android: {response}")

    except KeyboardInterrupt:
        print("\n[INFO] Shutting down (RFCOMM serial mode)...")
    finally:
        ser.close()


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

