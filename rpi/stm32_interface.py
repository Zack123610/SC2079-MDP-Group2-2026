"""
STM32-RPI USB Interface Module

Handles serial communication between Raspberry Pi and STM32F Board
via USB/UART interface (ttyACM0/ttyUSB0).
"""

import serial
import serial.tools.list_ports
import time
from typing import Optional


class STM32Interface:
    """
    Interface for communicating with STM32F Board via USB serial.
    
    The STM32F Board provides UART serial interface through its USB port
    running at baudrate 115200.
    """
    
    DEFAULT_BAUDRATE = 115200
    DEFAULT_TIMEOUT = 1.0  # seconds
    
    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = DEFAULT_BAUDRATE,
        timeout: float = DEFAULT_TIMEOUT
    ):
        """
        Initialize STM32 serial interface.
        
        Args:
            port: Serial port path (e.g., '/dev/ttyACM0'). 
                  If None, auto-detects the port.
            baudrate: Communication baudrate (default: 115200)
            timeout: Read timeout in seconds (default: 1.0)
        """
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        
        # Auto-detect or use specified port
        self.port = port if port else self._find_stm32_port()
        
        if self.port:
            self._connect()
        else:
            print("Warning: No STM32 device found. Call connect() manually after connecting the device.")
    
    def _find_stm32_port(self) -> Optional[str]:
        """
        Auto-detect STM32 serial port.
        
        Returns:
            Port path if found, None otherwise.
        """
        # Common patterns for STM32 USB serial
        patterns = ['ttyACM', 'ttyUSB']
        
        ports = serial.tools.list_ports.comports()
        for port in ports:
            for pattern in patterns:
                if pattern in port.device:
                    print(f"Found STM32 device at: {port.device}")
                    return port.device
        
        return None
    
    def _connect(self) -> bool:
        """
        Establish serial connection to STM32.
        
        Returns:
            True if connection successful, False otherwise.
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            # Give STM32 time to reset after connection
            time.sleep(0.1)
            print(f"Connected to STM32 at {self.port} (baudrate: {self.baudrate})")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to STM32: {e}")
            self.serial = None
            return False
    
    def connect(self, port: Optional[str] = None) -> bool:
        """
        Connect or reconnect to STM32.
        
        Args:
            port: Optional port path. If None, attempts auto-detection.
            
        Returns:
            True if connection successful, False otherwise.
        """
        if self.serial and self.serial.is_open:
            self.serial.close()
        
        if port:
            self.port = port
        else:
            self.port = self._find_stm32_port()
        
        if self.port:
            return self._connect()
        return False
    
    @property
    def is_connected(self) -> bool:
        """Check if serial connection is active."""
        return self.serial is not None and self.serial.is_open
    
    def send(self, message: str, add_newline: bool = True) -> bool:
        """
        Send a message to STM32.
        
        Args:
            message: Message string to send
            add_newline: Whether to append newline character (default: True)
            
        Returns:
            True if send successful, False otherwise.
        """
        if not self.is_connected:
            print("Error: Not connected to STM32")
            return False
        
        try:
            data = message
            # if add_newline and not message.endswith('\n'):
            #     data += '\n'
            
            self.serial.write(data.encode('utf-8'))
            self.serial.flush()
            print(f"Sent to STM32: {message}")
            return True
        except serial.SerialException as e:
            print(f"Send error: {e}")
            return False
    
    def receive(self, timeout: Optional[float] = None) -> Optional[str]:
        """
        Receive a message from STM32.
        
        Args:
            timeout: Optional timeout override in seconds
            
        Returns:
            Received message string, or None if timeout/error.
        """
        if not self.is_connected:
            print("Error: Not connected to STM32")
            return None
        
        try:
            if timeout is not None:
                old_timeout = self.serial.timeout
                self.serial.timeout = timeout
            
            line = self.serial.readline()
            
            if timeout is not None:
                self.serial.timeout = old_timeout
            
            if line:
                decoded = line.decode('utf-8').strip()
                print(f"Received from STM32: {decoded}")
                return decoded
            return None
        except serial.SerialException as e:
            print(f"Receive error: {e}")
            return None
    
    def receive_all(self, timeout: float = 0.5) -> list[str]:
        """
        Receive all available messages from STM32.
        
        Args:
            timeout: Timeout for each read attempt
            
        Returns:
            List of received messages.
        """
        messages = []
        while True:
            msg = self.receive(timeout=timeout)
            if msg is None:
                break
            messages.append(msg)
        return messages
    
    def send_raw(self, data: bytes) -> bool:
        """
        Send raw bytes to STM32 (no newline, no encoding).

        Useful for the 4-byte binary movement frames.

        Args:
            data: Raw bytes to send.

        Returns:
            True if send successful, False otherwise.
        """
        if not self.is_connected:
            print("Error: Not connected to STM32")
            return False

        try:
            self.serial.write(data)
            self.serial.flush()
            print(f"Sent raw to STM32: {data.hex()}")
            return True
        except serial.SerialException as e:
            print(f"Send raw error: {e}")
            return False

    def receive_raw(self, num_bytes: int = 4, timeout: Optional[float] = None) -> Optional[bytes]:
        """
        Receive an exact number of raw bytes from STM32.

        Useful for reading 4-byte binary status frames.

        Args:
            num_bytes: Number of bytes to read (default: 4).
            timeout:   Optional timeout override in seconds.

        Returns:
            bytes of length *num_bytes*, or None on timeout / error.
        """
        if not self.is_connected:
            print("Error: Not connected to STM32")
            return None

        try:
            if timeout is not None:
                old_timeout = self.serial.timeout
                self.serial.timeout = timeout

            data = self.serial.read(num_bytes)

            if timeout is not None:
                self.serial.timeout = old_timeout

            if data and len(data) == num_bytes:
                print(f"Received raw from STM32: {data.hex()}")
                return data
            return None
        except serial.SerialException as e:
            print(f"Receive raw error: {e}")
            return None

    def receive_all_raw(self, frame_size: int = 4) -> list[bytes]:
        """
        Read all complete raw frames currently buffered from STM32.

        This is non-blocking: it checks ``serial.in_waiting`` to see how
        many bytes are available and reads as many complete *frame_size*
        frames as possible without waiting.

        Args:
            frame_size: Size of one frame in bytes (default: 4).

        Returns:
            List of ``bytes`` objects, each of length *frame_size*.
            Empty list if nothing is available.
        """
        if not self.is_connected:
            return []

        frames: list[bytes] = []
        try:
            available = self.serial.in_waiting
            # Only read complete frames
            num_frames = available // frame_size
            for _ in range(num_frames):
                chunk = self.serial.read(frame_size)
                if chunk and len(chunk) == frame_size:
                    frames.append(chunk)
                else:
                    break
        except serial.SerialException as e:
            print(f"receive_all_raw error: {e}")

        if frames:
            print(f"Received {len(frames)} raw frame(s) from STM32")
        return frames

    def send_and_receive(
        self, 
        message: str, 
        timeout: Optional[float] = None
    ) -> Optional[str]:
        """
        Send a message and wait for response.
        
        Args:
            message: Message to send
            timeout: Optional timeout for response
            
        Returns:
            Response string, or None if timeout/error.
        """
        if self.send(message):
            return self.receive(timeout=timeout)
        return None
    
    def close(self):
        """Close the serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("STM32 connection closed")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
        return False


# Quick test when run directly
if __name__ == "__main__":
    print("STM32 Interface Test")
    print("-" * 40)
    
    # List available ports
    print("\nAvailable serial ports:")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f"  {port.device}: {port.description}")
    
    # Attempt connection
    print("\nAttempting to connect to STM32...")
    with STM32Interface() as stm:
        if stm.is_connected:
            print("\nConnection test successful!")
            print("Ready to send/receive data.")
            
            # Interactive test loop
            print("\nEnter messages to send (or 'quit' to exit):")
            while True:
                msg = input("> ")
                if msg.lower() == 'quit':
                    break
                stm.send(msg)
                response = stm.receive(timeout=10.0)
                if response:
                    print(f"Response: {response}")
        else:
            print("Could not connect to STM32. Check connection and try again.")
