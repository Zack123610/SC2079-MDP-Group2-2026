"""
Bluetooth Interface Module

Handles Bluetooth RFCOMM communication between the Raspberry Pi and the
Android tablet.

Two modes are supported:
  1. RFCOMM serial (default / recommended)
     A system service pre-binds the channel, exposing it as /dev/rfcomm0:
         sudo rfcomm listen /dev/rfcomm0 1
     This module then reads/writes over that serial device.

  2. Legacy PyBluez server (fallback)
     Python creates the RFCOMM server socket directly.
     Requires pybluez: pip install pybluez

Usage:
    bt = BluetoothInterface()          # RFCOMM serial mode (default)
    bt = BluetoothInterface(use_pybluez_server=True)  # legacy mode

    bt.start()            # open connection / start listening
    msg = bt.readline()   # read one line from Android (blocking with timeout)
    bt.send("OK")         # write a line back to Android
    bt.close()            # clean up

Context manager is supported:
    with BluetoothInterface() as bt:
        ...
"""

import sys
import threading
import time
from typing import Optional

import serial

# pybluez is optional; only needed for the legacy server mode.
try:
    import bluetooth  # type: ignore
    _PYBLUEZ_AVAILABLE = True
except ImportError:
    bluetooth = None  # type: ignore
    _PYBLUEZ_AVAILABLE = False


class BluetoothInterface:
    """
    Bluetooth RFCOMM interface for communicating with the Android tablet.

    Attributes
    ----------
    RFCOMM_DEVICE : str
        Serial device path used in RFCOMM serial mode (default: /dev/rfcomm0).
    RFCOMM_BAUDRATE : int
        Baud rate for the serial device (default: 115200).
    RFCOMM_CHANNEL : int
        Bluetooth RFCOMM channel number used in legacy PyBluez mode (default: 1).
    """

    RFCOMM_DEVICE = "/dev/rfcomm0"
    RFCOMM_BAUDRATE = 115200
    RFCOMM_CHANNEL = 1

    def __init__(
        self,
        use_pybluez_server: bool = False,
        device: Optional[str] = None,
        baudrate: Optional[int] = None,
        channel: Optional[int] = None,
    ) -> None:
        """
        Initialise the Bluetooth interface (does not connect yet).

        Args:
            use_pybluez_server: If True, use the legacy PyBluez RFCOMM server
                                 instead of the /dev/rfcomm0 serial device.
            device:    Override the RFCOMM serial device path.
            baudrate:  Override the serial baud rate.
            channel:   Override the PyBluez RFCOMM channel number.
        """
        self._use_pybluez = use_pybluez_server

        self._device = device or self.RFCOMM_DEVICE
        self._baudrate = baudrate or self.RFCOMM_BAUDRATE
        self._channel = channel or self.RFCOMM_CHANNEL

        # RFCOMM serial mode internals
        self._serial: Optional[serial.Serial] = None
        self._read_buf: bytes = b""

        # Legacy PyBluez server internals
        self._bt_server = None
        self._bt_client = None

    # -----------------------------------------------------------------------
    # Connection management
    # -----------------------------------------------------------------------

    def start(self) -> bool:
        """
        Open the Bluetooth connection.

        For RFCOMM serial mode: opens /dev/rfcomm0 as a serial port.
        For legacy PyBluez mode: creates the RFCOMM server and blocks until
        an Android device connects.

        Returns:
            True if the connection was established, False otherwise.
        """
        if self._use_pybluez:
            return self._start_pybluez()
        return self._start_rfcomm_serial()

    def _start_rfcomm_serial(self) -> bool:
        """Open /dev/rfcomm0 as a pyserial device (blocking reads)."""
        try:
            self._serial = serial.Serial(
                self._device,
                self._baudrate,
                timeout=1.0,
            )
            print(f"[BT] Opened {self._device} (baud={self._baudrate})")
            return True
        except serial.SerialException as e:
            print(f"[BT] Failed to open {self._device}: {e}")
            return False

    def _start_pybluez(self) -> bool:
        """Create a PyBluez RFCOMM server and wait for Android to connect."""
        if not _PYBLUEZ_AVAILABLE:
            print("[BT] pybluez is not installed. Cannot use legacy server mode.")
            return False

        self._bt_server = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self._bt_server.bind(("", self._channel))
        self._bt_server.listen(1)

        bluetooth.advertise_service(
            self._bt_server,
            "MDP-RPI",
            service_classes=[bluetooth.SERIAL_PORT_CLASS],
            profiles=[bluetooth.SERIAL_PORT_PROFILE],
        )

        print(f"[BT] Waiting for Android on RFCOMM channel {self._channel}...")
        self._bt_client, addr = self._bt_server.accept()
        print(f"[BT] Android connected: {addr}")
        return True

    def close(self) -> None:
        """Close the Bluetooth connection and release resources."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None
            print("[BT] Serial connection closed")

        if self._bt_client:
            try:
                self._bt_client.close()
            except Exception:
                pass
            self._bt_client = None
            print("[BT] Android client disconnected")

        if self._bt_server:
            try:
                self._bt_server.close()
            except Exception:
                pass
            self._bt_server = None
            print("[BT] Bluetooth server closed")

    @property
    def is_connected(self) -> bool:
        """True if the Bluetooth link is currently open."""
        if self._use_pybluez:
            return self._bt_client is not None
        return self._serial is not None and self._serial.is_open

    # -----------------------------------------------------------------------
    # Read / write
    # -----------------------------------------------------------------------

    def readline(self) -> Optional[str]:
        """
        Read one complete UTF-8 text line from Android.

        Lines are delimited by '\\n'. A partial line is buffered internally
        and returned once the delimiter arrives.

        Returns:
            Stripped message string, or None if nothing was available
            within the read timeout.
        """
        if self._use_pybluez:
            return self._readline_pybluez()
        return self._readline_serial()

    def _readline_serial(self) -> Optional[str]:
        """Read one line from the pyserial RFCOMM device."""
        if not (self._serial and self._serial.is_open):
            return None
        try:
            raw = self._serial.readline()
            if not raw:
                return None
            line = raw.decode("utf-8", errors="ignore").strip()
            return line if line else None
        except serial.SerialException as e:
            print(f"[BT] Read error: {e}")
            return None

    def _readline_pybluez(self) -> Optional[str]:
        """Read one newline-terminated line from the PyBluez client socket."""
        if not self._bt_client:
            return None
        try:
            chunk = self._bt_client.recv(1024)
            if not chunk:
                return None
            self._read_buf += chunk
            if b"\n" in self._read_buf:
                line, self._read_buf = self._read_buf.split(b"\n", 1)
                return line.decode("utf-8", errors="ignore").strip() or None
            return None
        except Exception as e:
            print(f"[BT] PyBluez recv error: {e}")
            return None

    def send(self, message: str) -> bool:
        """
        Send a text message to Android (a '\\n' is appended automatically).

        Args:
            message: Text string to send.

        Returns:
            True if sent successfully, False otherwise.
        """
        data = (message + "\n").encode("utf-8")

        if self._use_pybluez:
            return self._send_pybluez(data)
        return self._send_serial(data)

    def _send_serial(self, data: bytes) -> bool:
        if not (self._serial and self._serial.is_open):
            print("[BT] Not connected (serial)")
            return False
        try:
            self._serial.write(data)
            self._serial.flush()
            print(f"[BT] Sent to Android: {data.decode('utf-8', errors='ignore').strip()}")
            return True
        except serial.SerialException as e:
            print(f"[BT] Send error: {e}")
            return False

    def _send_pybluez(self, data: bytes) -> bool:
        if not self._bt_client:
            print("[BT] Not connected (PyBluez)")
            return False
        try:
            self._bt_client.send(data)
            print(f"[BT] Sent to Android: {data.decode('utf-8', errors='ignore').strip()}")
            return True
        except Exception as e:
            print(f"[BT] Send error: {e}")
            return False

    # -----------------------------------------------------------------------
    # Context manager
    # -----------------------------------------------------------------------

    def __enter__(self) -> "BluetoothInterface":
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        self.close()
        return False


# ---------------------------------------------------------------------------
# Bidirectional standalone test
# ---------------------------------------------------------------------------

def _receive_loop(bt: "BluetoothInterface", stop_event: threading.Event) -> None:
    """Background thread: print every line received from Android."""
    while not stop_event.is_set():
        msg = bt.readline()
        if msg:
            # Print on its own line, then reprint the input prompt so the
            # user's typing cursor stays clean.
            print(f"\n[Android -> Pi] {msg}")
            print("Send> ", end="", flush=True)
        else:
            time.sleep(0.05)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Bluetooth Interface – bidirectional test")
    parser.add_argument(
        "--pybluez", action="store_true",
        help="Use legacy PyBluez RFCOMM server instead of /dev/rfcomm0",
    )
    parser.add_argument(
        "--device", default=None,
        help="Override RFCOMM serial device (default: /dev/rfcomm0)",
    )
    args = parser.parse_args()

    print("Bluetooth Interface – Bidirectional Test")
    print("-" * 40)

    bt = BluetoothInterface(use_pybluez_server=args.pybluez, device=args.device)

    if not bt.start():
        print("Failed to connect. Check rfcomm setup and try again.")
        sys.exit(1)

    print("Connected.")
    print("  • Messages received from Android are printed automatically.")
    print("  • Type a message and press Enter to send it to Android.")
    print("  • Type 'quit' or press Ctrl+C to exit.")
    print()

    stop_event = threading.Event()
    recv_thread = threading.Thread(
        target=_receive_loop, args=(bt, stop_event), daemon=True
    )
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
