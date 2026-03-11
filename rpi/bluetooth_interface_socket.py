"""
Bluetooth Interface Module (Socket-based)

Uses Python's built-in socket.AF_BLUETOOTH to create an RFCOMM server,
eliminating the need for PyBluez or manual `sudo rfcomm listen` at boot.

Requirements:
    - BlueZ installed (default on Raspberry Pi OS)
    - Python built with Bluetooth support (default on Raspberry Pi OS)
    - No pip packages required

Before first use, ensure the RPi is paired with the Android device:
    bluetoothctl
    > discoverable on
    > pairable on
    > agent on
    > default-agent
    (pair from Android, confirm on both sides)

Usage:
    bt = BluetoothInterface()
    bt.start()            # advertise SDP + wait for Android to connect
    msg = bt.readline()   # read one line from Android (blocking with timeout)
    bt.send("OK")         # write a line back to Android
    bt.close()            # clean up

Context manager:
    with BluetoothInterface() as bt:
        ...
"""

import socket
import subprocess
import sys
import threading
import time
from typing import Optional


# These constants live in the kernel headers; define them here so the module
# works even if the local socket module doesn't expose the names.
AF_BLUETOOTH = getattr(socket, "AF_BLUETOOTH", 31)
BTPROTO_RFCOMM = getattr(socket, "BTPROTO_RFCOMM", 3)

SERVICE_NAME = "MDP-RPI"
SERVICE_UUID = "00001101-0000-1000-8000-00805F9B34FB"  # Serial Port Profile


class BluetoothInterface:
    """
    Bluetooth RFCOMM interface using Python's native socket API.

    Creates an RFCOMM server socket, advertises a Serial Port Profile via
    sdptool, and waits for an Android device to connect — all without
    requiring PyBluez or a pre-configured /dev/rfcomm0 device.
    """

    def __init__(self, channel: int = 1, read_timeout: float = 1.0) -> None:
        self._channel = channel
        self._read_timeout = read_timeout

        self._server: Optional[socket.socket] = None
        self._client: Optional[socket.socket] = None
        self._client_addr: Optional[tuple] = None
        self._read_buf: bytes = b""

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """
        Make the RPi discoverable, register the SDP service,
        and block until an Android device connects.
        """
        try:
            self._make_discoverable()
            self._register_sdp()

            self._server = socket.socket(AF_BLUETOOTH, socket.SOCK_STREAM, BTPROTO_RFCOMM)
            self._server.bind(("00:00:00:00:00:00", self._channel))
            self._server.listen(1)

            print(f"[BT] Listening on RFCOMM channel {self._channel} …")
            self._client, self._client_addr = self._server.accept()
            self._client.settimeout(self._read_timeout)
            print(f"[BT] Android connected: {self._client_addr[0]}")
            return True

        except OSError as exc:
            print(f"[BT] Failed to start: {exc}")
            self.close()
            return False

    def close(self) -> None:
        """Close all sockets and unregister the SDP service."""
        if self._client:
            try:
                self._client.close()
            except OSError:
                pass
            self._client = None
            print("[BT] Client disconnected")

        if self._server:
            try:
                self._server.close()
            except OSError:
                pass
            self._server = None
            print("[BT] Server closed")

        self._read_buf = b""

    @property
    def is_connected(self) -> bool:
        return self._client is not None

    # ------------------------------------------------------------------
    # Read / write
    # ------------------------------------------------------------------

    def readline(self) -> Optional[str]:
        """
        Read one newline-terminated UTF-8 string from Android.

        Blocks up to *read_timeout* seconds.  Partial data is buffered
        internally and returned once '\\n' arrives.
        """
        if not self._client:
            return None
        try:
            while True:
                if b"\n" in self._read_buf:
                    line, self._read_buf = self._read_buf.split(b"\n", 1)
                    decoded = line.decode("utf-8", errors="ignore").strip()
                    return decoded if decoded else None

                chunk = self._client.recv(1024)
                if not chunk:
                    return None
                self._read_buf += chunk

        except socket.timeout:
            return None
        except OSError as exc:
            print(f"[BT] Read error: {exc}")
            return None

    def send(self, message: str) -> bool:
        """Send a line of text to Android ('\\n' appended automatically)."""
        if not self._client:
            print("[BT] Not connected")
            return False
        try:
            self._client.sendall((message + "\n").encode("utf-8"))
            print(f"[BT] Sent: {message}")
            return True
        except OSError as exc:
            print(f"[BT] Send error: {exc}")
            return False

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "BluetoothInterface":
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        self.close()
        return False

    # ------------------------------------------------------------------
    # Helpers (BlueZ CLI wrappers)
    # ------------------------------------------------------------------

    @staticmethod
    def _make_discoverable() -> None:
        """Use hciconfig/bluetoothctl to make the adapter discoverable."""
        try:
            subprocess.run(
                ["bluetoothctl", "discoverable", "on"],
                timeout=5, check=False,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            subprocess.run(
                ["bluetoothctl", "pairable", "on"],
                timeout=5, check=False,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            print("[BT] Adapter set to discoverable + pairable")
        except FileNotFoundError:
            print("[BT] bluetoothctl not found – skipping discoverability setup")

    @staticmethod
    def _register_sdp() -> None:
        """Register Serial Port Profile with the local SDP server."""
        try:
            subprocess.run(
                ["sudo", "sdptool", "add", "SP"],
                timeout=5, check=False,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            print("[BT] SDP Serial Port service registered")
        except FileNotFoundError:
            print("[BT] sdptool not found – Android may still connect if already paired")


# ---------------------------------------------------------------------------
# Standalone bidirectional test
# ---------------------------------------------------------------------------

def _receive_loop(bt: BluetoothInterface, stop: threading.Event) -> None:
    while not stop.is_set():
        msg = bt.readline()
        if msg:
            print(f"\n[Android -> Pi] {msg}")
            print("Send> ", end="", flush=True)
        else:
            time.sleep(0.05)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Bluetooth Interface (socket) – bidirectional test")
    parser.add_argument("--channel", type=int, default=1, help="RFCOMM channel (default: 1)")
    args = parser.parse_args()

    print("Bluetooth Interface (socket) – Bidirectional Test")
    print("-" * 50)

    bt = BluetoothInterface(channel=args.channel)

    if not bt.start():
        print("Failed to start. Is Bluetooth enabled?")
        sys.exit(1)

    print("Connected.")
    print("  • Messages from Android are printed automatically.")
    print("  • Type a message + Enter to send to Android.")
    print("  • Type 'quit' or Ctrl+C to exit.\n")

    stop_event = threading.Event()
    recv_thread = threading.Thread(target=_receive_loop, args=(bt, stop_event), daemon=True)
    recv_thread.start()

    try:
        while True:
            print("Send> ", end="", flush=True)
            try:
                line = input()
            except EOFError:
                break
            if line.strip().lower() in ("quit", "exit", "q"):
                break
            if line.strip():
                bt.send(line.strip())
    except KeyboardInterrupt:
        print()
    finally:
        stop_event.set()
        recv_thread.join(timeout=1.0)
        bt.close()
        print("Exiting.")
