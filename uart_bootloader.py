#!/usr/bin/env python3
"""
UART Bootloader — STM32/GD32 UART bootloader protocol implementation.

Uses the same protocol as the CAN bootloader, but over UART (serial).
Inherits from BaseBootloader which implements all protocol logic.

Uses pyserial for USB-UART communication.

Dependencies: pyserial, intelhex
"""

from __future__ import annotations

import logging
from typing import Optional

import serial

from can_bootloader import (
    BaseBootloader,
    BootloaderError, NackError, TimeoutError,
    RESPONSE_TIMEOUT,
    ChipInfo, lookup_chip,
)

logger = logging.getLogger(__name__)

# Timeouts (seconds)
CONNECTION_TIMEOUT = 10.0
DEFAULT_BAUDRATE = 115200


class UARTBootloader(BaseBootloader):
    """
    UART bootloader for STM32/GD32.

    Uses pyserial for communication over USB-UART adapter.

    Usage:
        bl = UARTBootloader(port='COM3', baudrate=115200)
        bl.connect()
        try:
            bl.sync()
            info = bl.get()
            chip_id = bl.get_id()
            bl.erase_memory()
            bl.program_hex('firmware.hex')
        finally:
            bl.disconnect()
    """

    def __init__(self, port: str = 'COM3', baudrate: int = DEFAULT_BAUDRATE,
                 parity: str = 'E'):
        self.port = port
        self.baudrate = baudrate
        self.parity = parity
        self.ser: Optional[serial.Serial] = None
        self._current_timeout: Optional[float] = None

    @staticmethod
    def _parity_str_to_serial(parity: str) -> str:
        """Map parity string ('NONE', 'EVEN', 'ODD') to pyserial constant."""
        m = {'NONE': serial.PARITY_NONE,
             'EVEN': serial.PARITY_EVEN,
             'ODD':  serial.PARITY_ODD}
        return m.get(parity.upper(), serial.PARITY_EVEN)

    # ─── Low-level UART I/O ──────────────────────────────────────────────

    def _send_byte(self, byte: int) -> None:
        """Send one byte over UART."""
        assert self.ser is not None, "Serial port not connected"
        self.ser.write(bytes([byte]))

    def _recv_byte(self, timeout: float = RESPONSE_TIMEOUT) -> int:
        """
        Receive one byte from UART.

        Raises TimeoutError if no byte received within timeout.
        """
        assert self.ser is not None, "Serial port not connected"
        # Cache timeout to avoid expensive SetCommTimeouts() syscall on
        # every byte (Windows + USB-UART: ~12ms per set).
        if self._current_timeout != timeout:
            self.ser.timeout = timeout
            self._current_timeout = timeout
        data = self.ser.read(1)
        if not data:
            raise TimeoutError("No response from device")
        return data[0]

    # ─── Connection management ───────────────────────────────────────────

    def connect(self) -> None:
        """Connect to the UART port."""
        logger.info(f"Connecting to {self.port} at {self.baudrate} bps")
        parity_val = self._parity_str_to_serial(self.parity)
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=parity_val,
            stopbits=serial.STOPBITS_ONE,
            timeout=RESPONSE_TIMEOUT,
        )
        logger.info("Serial port connected")

    def set_baudrate(self, baudrate: int) -> None:
        """Change baudrate on the fly (reconfigures the open serial port)."""
        assert self.ser is not None, "Serial port not connected"
        self.ser.baudrate = baudrate
        logger.info(f"Baudrate changed to {baudrate}")

    def disconnect(self) -> None:
        """Disconnect from the UART port."""
        if self.ser is not None:
            self.ser.close()
            self.ser = None
            logger.info("Serial port disconnected")
