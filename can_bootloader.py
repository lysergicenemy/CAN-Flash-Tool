#!/usr/bin/env python3
"""
CAN Bootloader — STM32 CAN bootloader protocol implementation for GD32/STM32.

Adaptation of the STM32 UART bootloader protocol for CAN bus.
- CAN ID request (PC→MCU):   0x3
- CAN ID response (MCU→PC):  0x4
- DLC: always 8 bytes (standard CAN frame), but only 1 byte is used (first byte)
- Each CAN frame carries exactly 1 byte of data in its first byte.

Uses python-can with the 'candle' interface (python-can-candle package)
for USB-CAN adapters based on the candleLight firmware (gs_usb compatible).

Dependencies: python-can, python-can-candle, intelhex
"""

from __future__ import annotations

import abc
import time
import logging
from dataclasses import dataclass
from typing import List, Optional, Tuple, Callable

import can
from intelhex import IntelHex

# ─── Force candle interface registration ─────────────────────────────────
# The python-can-candle package registers itself via entry points.
# PyInstaller does NOT automatically include entry-point-based modules,
# so we must register the interface manually in can.interfaces.BACKENDS.
# In frozen builds (PyInstaller), this ensures can.Bus(interface='candle')
# knows where to find CandleBus.

# Explicitly import candle and its native dependency (candle_api with .pyd)
# so PyInstaller includes them in frozen builds.
try:
    import candle  # noqa: F401 — entire candle package
    import candle.candle_bus  # noqa: F401 — CandleBus class (the actual bus)
    import candle_api  # noqa: F401 — native .pyd extension (C bindings)
except ImportError:
    pass

if 'candle' not in can.interfaces.BACKENDS:
    can.interfaces.BACKENDS['candle'] = ('candle.candle_bus', 'CandleBus')

logger = logging.getLogger(__name__)

# ─── Protocol constants ─────────────────────────────────────────────────────

SYNC_BYTE = 0x7F
ACK_BYTE = 0x79
NACK_BYTE = 0x1F

# Commands
CMD_GET = 0x00
CMD_GET_ID = 0x02
CMD_READ_MEMORY = 0x11
CMD_WRITE_MEMORY = 0x31
CMD_ERASE_MEMORY = 0x43
CMD_GO = 0x21

# Timeouts (seconds)
RESPONSE_TIMEOUT = 2.0
CONNECTION_TIMEOUT = 10.0

# Maximum write block size (protocol limit)
MAX_WRITE_SIZE = 256

# ─── Chip database ──────────────────────────────────────────────────────────
@dataclass
class ChipInfo:
    """Information about a specific MCU chip."""
    name: str           # e.g. "GD32E230"
    page_size: int      # flash page size in bytes (e.g. 0x400 = 1 KB)
    pages: int          # total number of flash pages (e.g. 32 for 32 KB)


CHIP_DATABASE: dict[int, ChipInfo] = {
    0x0444: ChipInfo("GD32E230",  page_size=0x400,  pages=16),   # 16 KB
    0x0448: ChipInfo("GD32F103",  page_size=0x400,  pages=64),   # 64 KB
    0x0442: ChipInfo("GD32F303CB",  page_size=0x800,  pages=64), # 128 KB
    0x0410: ChipInfo("STM32F103", page_size=0x400,  pages=64),   # 64 KB
    0x0441: ChipInfo("GD32E103",  page_size=0x400,  pages=64),   # 64 KB
    0x0430: ChipInfo("GD32F130",  page_size=0x400,  pages=32),   # 32 KB
    0x0446: ChipInfo("GD32F450",  page_size=0x800,  pages=256),  # 512 KB
    0x0450: ChipInfo("GD32E508",  page_size=0x800,  pages=128),  # 256 KB
    0x0452: ChipInfo("GD32VF103", page_size=0x400,  pages=64),   # 64 KB
    0x0449: ChipInfo("GD32F101",  page_size=0x400,  pages=32),   # 32 KB
    0x0443: ChipInfo("GD32F103C", page_size=0x400,  pages=128),  # 128 KB
    0x0420: ChipInfo("STM32F407", page_size=0x800,  pages=256),  # 512 KB
    0x0434: ChipInfo("STM32F030", page_size=0x400,  pages=32),   # 32 KB
    0x0447: ChipInfo("GD32F105",  page_size=0x400,  pages=64),   # 64 KB
}


def lookup_chip(chip_id: int) -> ChipInfo:
    """Lookup chip information by chip ID.

    Returns ChipInfo with name=f"Unknown(0x{chip_id:04X})"
    and page_size/pages = 0 if the chip ID is not in the database.
    """
    return CHIP_DATABASE.get(
        chip_id,
        ChipInfo(name=f"Unknown(0x{chip_id:04X})", page_size=0, pages=0)
    )


class BootloaderError(Exception):

    """Base bootloader exception."""
    pass


class NackError(BootloaderError):
    """Device responded with NACK."""
    def __init__(self, context: str = ""):
        super().__init__(f"Device responded NACK {context}".strip())


class TimeoutError(BootloaderError):
    """Response timeout."""
    pass


# ─── Abstract Base Bootloader ─────────────────────────────────────────────

class BaseBootloader(abc.ABC):
    """
    Abstract base class implementing the STM32/GD32 bootloader protocol.

    Subclasses must implement the transport-specific methods:
      - _send_byte(byte)
      - _recv_byte(timeout)
      - connect()
      - disconnect()

    All high-level protocol commands (sync, get, get_id, read_memory,
    write_memory, erase_memory, go, ping) are implemented here.
    """

    @abc.abstractmethod
    def _send_byte(self, byte: int) -> None:
        """Send one byte to the device."""

    @abc.abstractmethod
    def _recv_byte(self, timeout: float = RESPONSE_TIMEOUT) -> int:
        """Receive one byte from the device."""

    def _recv_ack(self) -> None:
        """Receive and verify ACK byte (or raise on NACK)."""
        byte = self._recv_byte()
        if byte == NACK_BYTE:
            raise NackError()
        if byte != ACK_BYTE:
            raise BootloaderError(f"Expected ACK (0x{ACK_BYTE:02X}), got 0x{byte:02X}")

    def _send_cmd(self, cmd: int) -> None:
        """
        Send command: command byte + its complement (two bytes).
        Waits for ACK/NACK after sending.
        """
        self._send_byte(cmd)
        self._send_byte((~cmd) & 0xFF)
        self._recv_ack()

    # ─── Connection management ───────────────────────────────────────────

    @abc.abstractmethod
    def connect(self) -> None:
        """Connect to the device."""

    @abc.abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the device."""

    # ─── High-level commands ─────────────────────────────────────────────

    def sync(self) -> None:
        """
        Synchronize with the bootloader.
        Sends 0x7F, expects ACK.
        """
        logger.info("Sending sync (0x7F)")
        self._send_byte(SYNC_BYTE)
        self._recv_ack()
        logger.info("Sync OK")

    def get(self) -> Tuple[int, List[int]]:
        """
        Get command (0x00) — get bootloader version and supported commands.

        Returns:
            (version, list_of_supported_commands)
        """
        logger.info("CMD_GET")
        self._send_cmd(CMD_GET)

        # Receive N (number of following bytes - 1)
        n = self._recv_byte()

        # Receive N+1 bytes: version + commands
        total = n + 1
        data = []
        for _ in range(total):
            data.append(self._recv_byte())

        # ACK comes AFTER the data bytes, not as part of them
        self._recv_ack()

        version = data[0]
        commands = data[1:]  # all remaining bytes are commands
        logger.info(f"  Version: 0x{version:02X}, Commands: {[f'0x{c:02X}' for c in commands]}")
        return version, commands

    def get_id(self) -> int:
        """
        Get ID command (0x02) — get chip ID.

        Returns:
            Chip ID (2 bytes, e.g. 0x0440 for GD32E230)
        """
        logger.info("CMD_GET_ID")
        self._send_cmd(CMD_GET_ID)

        n = self._recv_byte()  # number of ID bytes - 1
        total = n + 1           # number of ID bytes
        data = []
        for _ in range(total):
            data.append(self._recv_byte())

        # ACK comes AFTER the ID bytes
        self._recv_ack()

        chip_id = (data[0] << 8) | data[1]
        logger.info(f"  Chip ID: 0x{chip_id:04X}")
        return chip_id

    def ping(self, timeout: float = 0.5) -> bool:
        """
        Ping the device by performing GET_ID with short timeout.

        Two attempts are made — if both fail, device is considered
        disconnected.

        Args:
            timeout: per-byte receive timeout (seconds)

        Returns:
            True if device responded, False otherwise.
        """
        for attempt in range(2):
            try:
                self._send_cmd(CMD_GET_ID)
                n = self._recv_byte(timeout=timeout)
                total = n + 1
                for _ in range(total):
                    self._recv_byte(timeout=timeout)
                self._recv_ack()
                logger.info("Ping OK")
                return True
            except (TimeoutError, NackError):
                logger.debug(f"Ping attempt {attempt + 1} failed")
                if attempt == 1:
                    return False
                continue
        return False

    def read_memory(self, address: int, size: int) -> bytes:
        """
        Read Memory command (0x11) — read memory content.

        Args:
            address: start address
            size: number of bytes to read

        Returns:
            read data
        """
        logger.info(f"CMD_READ_MEMORY: addr=0x{address:08X}, size={size}")
        self._send_cmd(CMD_READ_MEMORY)

        # Phase 1: address (big-endian, 4 bytes) + XOR checksum
        addr_bytes = address.to_bytes(4, 'big')
        xor_addr = self._xor_checksum(addr_bytes)
        for b in addr_bytes:
            self._send_byte(b)
        self._send_byte(xor_addr)
        self._recv_ack()

        # Phase 2: N (size-1) + ~N
        n = size - 1
        self._send_byte(n)
        self._send_byte((~n) & 0xFF)
        self._recv_ack()

        # Receive data: N+1 bytes
        result = bytearray()
        for _ in range(size):
            result.append(self._recv_byte())

        logger.info(f"  Read {len(result)} bytes")
        return bytes(result)

    def write_memory(self, address: int, data: bytes) -> None:
        """
        Write Memory command (0x31) — write memory.

        Args:
            address: start address (must be aligned)
            data: data to write (up to 256 bytes)
        """
        assert len(data) <= MAX_WRITE_SIZE, \
            f"Data too large: {len(data)} > {MAX_WRITE_SIZE}"

        logger.info(f"CMD_WRITE_MEMORY: addr=0x{address:08X}, size={len(data)}")
        self._send_cmd(CMD_WRITE_MEMORY)

        # Phase 1: address (big-endian, 4 bytes) + XOR
        addr_bytes = address.to_bytes(4, 'big')
        xor_addr = self._xor_checksum(addr_bytes)
        for b in addr_bytes:
            self._send_byte(b)
        self._send_byte(xor_addr)
        self._recv_ack()

        # Phase 2: N (size-1) + data + XOR
        n = len(data) - 1
        payload = bytes([n]) + data
        xor_data = self._xor_checksum(payload)
        for b in payload:
            self._send_byte(b)
        self._send_byte(xor_data)
        self._recv_ack()

        logger.info(f"  Written {len(data)} bytes")

    def erase_memory(self, global_erase: bool = True,
                     pages: Optional[List[int]] = None) -> None:
        """
        Erase Memory command (0x43) — erase flash memory.

        Args:
            global_erase: if True — mass erase
            pages: list of page numbers for selective erase
        """
        logger.info("CMD_ERASE_MEMORY")
        self._send_cmd(CMD_ERASE_MEMORY)

        if global_erase:
            # N=0xFF + XOR
            payload = bytes([0xFF, 0x00])
            for b in payload:
                self._send_byte(b)
        else:
            assert pages is not None, "pages required for page erase"
            n = len(pages) - 1
            payload = bytes([n] + pages)
            xor_val = self._xor_checksum(payload)
            for b in payload:
                self._send_byte(b)
            self._send_byte(xor_val)

        self._recv_ack()
        logger.info("  Erase OK")

    def go(self, address: int) -> None:
        """
        Go command (0x21) — start application at given address.

        Args:
            address: start address (typically APP_START_ADDR)
        """
        logger.info(f"CMD_GO: addr=0x{address:08X}")
        self._send_cmd(CMD_GO)

        # Address big-endian + XOR
        addr_bytes = address.to_bytes(4, 'big')
        xor_addr = self._xor_checksum(addr_bytes)
        for b in addr_bytes:
            self._send_byte(b)
        self._send_byte(xor_addr)

        self._recv_ack()
        logger.info("  GO command sent, device should restart")

    # ─── Utilities ───────────────────────────────────────────────────────

    @staticmethod
    def _xor_checksum(data: bytes) -> int:
        """Compute XOR checksum of bytes."""
        result = 0
        for b in data:
            result ^= b
        return result

    # ─── HEX file operations ─────────────────────────────────────────────

    def program_hex(self, hex_path: str,
                    progress_callback: Optional[Callable] = None) -> None:
        """
        Program device from HEX file.

        Args:
            hex_path: path to .hex file
            progress_callback: callback(progress, status) for progress indication
        """
        logger.info(f"Loading hex file: {hex_path}")
        ih = IntelHex(hex_path)
        segments = ih.segments()
        logger.info(f"  Segments: {segments}")

        total_size = sum(end - start for start, end in segments)
        written = 0

        for start_addr, end_addr in segments:
            addr = start_addr
            while addr < end_addr:
                chunk_size = min(MAX_WRITE_SIZE, end_addr - addr)
                chunk = ih.tobinstr(addr, addr + chunk_size - 1)

                if isinstance(chunk, str):
                    chunk = chunk.encode('latin-1')

                self.write_memory(addr, chunk)

                written += len(chunk)
                if progress_callback:
                    progress = int(written * 100 / total_size) if total_size > 0 else 0
                    progress_callback(progress, f"Writing 0x{addr:08X}...")

                addr += chunk_size

        logger.info("Programming complete")
        if progress_callback:
            progress_callback(100, "Done!")

    def read_to_hex(self, address: int, size: int, hex_path: str,
                    chunk_size: int = 256,
                    progress_callback: Optional[Callable] = None) -> None:
        """
        Read device memory and save to HEX file.

        Args:
            address: start address
            size: number of bytes to read
            hex_path: output .hex file path
            chunk_size: read block size (max 256)
            progress_callback: callback(progress, status)
        """
        logger.info(f"Reading memory to {hex_path}")
        ih = IntelHex()
        read_bytes = 0

        addr = address
        while addr < address + size:
            remaining = address + size - addr
            cur_size = min(chunk_size, remaining)
            data = self.read_memory(addr, cur_size)
            ih[addr:addr + cur_size] = list(data)

            read_bytes += cur_size
            addr += cur_size

            if progress_callback:
                progress = int(read_bytes * 100 / size)
                progress_callback(progress, f"Reading 0x{addr:08X}...")

        ih.write_hex_file(hex_path)
        logger.info(f"Saved to {hex_path}")
        if progress_callback:
            progress_callback(100, "Done!")


# ─── CAN Bootloader ────────────────────────────────────────────────────────

# CAN IDs
CAN_ID_HOST = 0x3    # PC → MCU
CAN_ID_DEVICE = 0x4  # MCU → PC
DEFAULT_BITRATE = 250000

# Inter-byte spacing (milliseconds) — small delay between each CAN frame sent
# to prevent saturating the CAN bus during bootloader operations.
# Other devices on the bus will still be able to communicate.
SEND_INTERVAL_MS = 2


class CANBootloader(BaseBootloader):
    """
    CAN bootloader for STM32/GD32 (1 byte per CAN frame).

    Uses the 'candle' python-can interface for USB-CAN adapters
    based on candleLight firmware.

    Usage:
        bl = CANBootloader(channel=0, bitrate=125000)
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

    def __init__(self, interface: str = 'candle', channel: int = 0,
                 bitrate: int = DEFAULT_BITRATE):
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.bus: Optional[can.BusABC] = None

    # ─── Low-level CAN I/O ──────────────────────────────────────────────

    def _flush_rx_buffer(self) -> int:
        """
        Flush all pending CAN messages from the device (ID=0x04).

        Non-blocking reads with timeout=0 to drain stale messages
        that may have accumulated before sync (e.g. after a device
        reset triggered by a pre-handshake message).

        Returns:
            Number of flushed messages.
        """
        assert self.bus is not None, "Bus not connected"
        flushed = 0
        while True:
            msg = self.bus.recv(timeout=0)  # non-blocking
            if msg is None:
                break
            if msg.arbitration_id == CAN_ID_DEVICE and msg.dlc != 1:
                flushed += 1
                logger.debug(f"Flushed stale message from device: "
                             f"ID=0x{msg.arbitration_id:X}, DLC={msg.dlc}, data={msg.data.hex()}")
        if flushed:
            logger.info(f"Flushed {flushed} stale message(s) from RX buffer")
        return flushed

    def _send_byte(self, byte: int) -> None:
        """Send one byte as a single CAN frame (DLC=1)."""
        assert self.bus is not None, "Bus not connected"
        msg = can.Message(
            arbitration_id=CAN_ID_HOST,
            data=bytes([byte]),
            is_extended_id=False,
            dlc=1,
        )
        self.bus.send(msg)
        # Small delay between messages to avoid saturating the CAN bus
        # so other devices on the bus can still communicate.
        if SEND_INTERVAL_MS > 0:
            time.sleep(SEND_INTERVAL_MS / 1000.0)

    def _recv_byte(self, timeout: float = RESPONSE_TIMEOUT) -> int:
        """
        Receive one CAN frame from device and return its single byte.
        Filters for CAN_ID_DEVICE and DLC=1 (bootloader protocol uses
        exactly 1 data byte per CAN frame; stale messages with DLC>1
        are silently skipped).
        """
        assert self.bus is not None, "Bus not connected"
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("No response from device")
            msg = self.bus.recv(remaining)
            if msg is None:
                continue
            if msg.arbitration_id == CAN_ID_DEVICE:
                # Skip stale messages (DLC != 1) — typically 8-byte zero
                # frames that some MCUs emit after CAN peripheral reset
                if msg.dlc != 1:
                    logger.debug(f"Skipping stale frame: DLC={msg.dlc}, "
                                 f"data={msg.data.hex()}")
                    continue
                # DLC=1, first byte is our data
                return msg.data[0]
        raise TimeoutError("No response from device")

    # ─── High-level CAN commands ────────────────────────────────────────

    def sync(self) -> None:
        """
        Synchronize with the bootloader.
        Flushes stale messages first, then sends 0x7F, expects ACK.
        """
        # Flush any stale messages that arrived before sync
        # (e.g. after a device reset triggered by pre-handshake)
        self._flush_rx_buffer()

        logger.info("Sending sync (0x7F)")
        self._send_byte(SYNC_BYTE)
        self._recv_ack()
        logger.info("Sync OK")

    # ─── Connection management ───────────────────────────────────────────

    def connect(self) -> None:
        """Connect to the CAN interface."""
        logger.info(f"Connecting to {self.interface}:{self.channel} at {self.bitrate} bps")
        self.bus = can.Bus(
            interface=self.interface,
            channel=self.channel,
            bitrate=self.bitrate,
        )
        logger.info("CAN bus connected")


    def disconnect(self) -> None:
        """Disconnect from the CAN interface."""
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None
            logger.info("CAN bus disconnected")


# ─── CLI usage example ──────────────────────────────────────────────────────

def main():
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(message)s")

    bl = CANBootloader(channel=0, bitrate=125000)
    bl.connect()

    try:
        bl.sync()

        version, commands = bl.get()
        print(f"Bootloader version: 0x{version:02X}")
        print(f"Supported commands: {[hex(c) for c in commands]}")

        chip_id = bl.get_id()
        print(f"Chip ID: 0x{chip_id:04X}")

        data = bl.read_memory(0x08000000, 16)
        print(f"Memory at 0x08000000: {' '.join(f'{b:02X}' for b in data)}")

    except BootloaderError as e:
        print(f"Error: {e}")
    finally:
        bl.disconnect()


if __name__ == '__main__':
    main()
