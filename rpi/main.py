"""
Raspberry Pi Main Program
Receives messages from Android via Bluetooth and forwards to STM32.

Run on Raspberry Pi:
    python main.py

Dependencies:
    pip install pyserial pybluez

Setup Bluetooth:
    sudo apt install bluetooth bluez libbluetooth-dev
    sudo systemctl enable bluetooth
    sudo systemctl start bluetooth
"""

import sys
import bluetooth
from stm32_interface import STM32Interface


def setup_bluetooth(channel: int = 1) -> bluetooth.BluetoothSocket:
    """Setup Bluetooth RFCOMM server socket."""
    server = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    server.bind(("", channel))
    server.listen(1)

    # Make discoverable
    bluetooth.advertise_service(
        server,
        "MDP-RPI",
        service_classes=[bluetooth.SERIAL_PORT_CLASS],
        profiles=[bluetooth.SERIAL_PORT_PROFILE]
    )
    
    print(f"[BT] Waiting for Android connection on channel {channel}...")
    return server


def main():
    # Connect to STM32
    print("[STM32] Connecting...")
    stm = STM32Interface()
    if not stm.is_connected:
        print("[STM32] Failed to connect. Exiting.")
        sys.exit(1)
    print("[STM32] Connected")

    # Setup Bluetooth server
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
        print("\n[INFO] Shutting down...")
    finally:
        bt_server.close()
        stm.close()


if __name__ == "__main__":
    main()
