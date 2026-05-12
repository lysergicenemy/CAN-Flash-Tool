#!/usr/bin/env python3
"""
CAN Bootloader Simulator — эмулирует GD32/STM32 bootloader на виртуальной CAN шине.

Протокол: 1 байт = 1 CAN фрейм (DLC=1) — как UART over CAN.

Используется для отладки CAN Flash Tool GUI без реального железа.

Запуск:
    python can_bootloader_sim.py

В другом терминале:
    python can_flash_gui.py
    → Interface: virtual, Channel: test_bus, Bitrate: 125000
    → Connect

Зависимости: python-can, intelhex
"""

from __future__ import annotations

import logging
import time
from typing import Optional, List, Tuple

import can

logger = logging.getLogger(__name__)

# ─── Protocol constants (совпадают с can_bootloader.py) ─────────────────────

SYNC_BYTE = 0x7F
ACK_BYTE = 0x79
NACK_BYTE = 0x1F

CMD_GET = 0x00
CMD_GET_VERSION = 0x01
CMD_GET_ID = 0x02
CMD_READ_MEMORY = 0x11
CMD_WRITE_MEMORY = 0x31
CMD_ERASE_MEMORY = 0x43
CMD_GO = 0x21

CAN_ID_HOST = 0x3      # PC → MCU
CAN_ID_DEVICE = 0x4    # MCU → PC

FLASH_SIZE = 64 * 1024  # 64 KB виртуальной Flash
FLASH_BASE = 0x08000000 # Базовый адрес Flash (STM32/GD32)


class CANBootloaderSimulator:
    """
    Симулятор CAN bootloader'а (1 байт = 1 CAN фрейм).

    Эмулирует:
    - SYNC (0x7F) → ACK
    - CMD_GET (0x00) → версия + список команд
    - CMD_GET_ID (0x02) → Chip ID (GD32E230 = 0x0440)
    - CMD_READ_MEMORY (0x11) → чтение из виртуального flash-массива
    - CMD_WRITE_MEMORY (0x31) → запись в виртуальный flash-массив
    - CMD_ERASE_MEMORY (0x43) → стирание (глобальное или страниц)
    - CMD_GO (0x21) → выход (остановка симулятора)
    """

    def __init__(self, channel: str = "test_bus", bitrate: int = 125000,
                 chip_id: int = 0x0440, version: int = 0x22):
        self.channel = channel
        self.bitrate = bitrate
        self.chip_id = chip_id
        self.version = version
        self.bus: Optional[can.BusABC] = None

        # Виртуальная Flash-память
        self.flash = bytearray(FLASH_SIZE)
        self.flash[:] = b'\xFF' * FLASH_SIZE

        # Список поддерживаемых команд
        self.supported_commands = [
            CMD_GET,
            CMD_GET_VERSION,
            CMD_GET_ID,
            CMD_READ_MEMORY,
            CMD_WRITE_MEMORY,
            CMD_ERASE_MEMORY,
            CMD_GO,
        ]

        self.running = False

    def _flash_to_offset(self, address: int) -> int:
        """Преобразовать абсолютный адрес Flash в смещение в массиве."""
        if address >= FLASH_BASE and address < FLASH_BASE + FLASH_SIZE:
            return address - FLASH_BASE
        if 0 <= address < FLASH_SIZE:
            return address
        return -1

    # ─── CAN I/O (1 байт = 1 фрейм, DLC=1) ──────────────────────────────

    def _send_byte(self, byte: int) -> None:
        """Отправить 1 байт как CAN фрейм (DLC=1, ID=DEVICE)."""
        assert self.bus is not None
        msg = can.Message(
            arbitration_id=CAN_ID_DEVICE,
            data=bytes([byte]),
            is_extended_id=False,
            dlc=1,
        )
        self.bus.send(msg)

    def _send_ack(self) -> None:
        """Отправить ACK."""
        self._send_byte(ACK_BYTE)

    def _send_nack(self) -> None:
        """Отправить NACK."""
        self._send_byte(NACK_BYTE)

    def _recv_byte(self) -> int:
        """
        Принять 1 байт из CAN фрейма (ID=HOST, DLC=1).
        Возвращает -1 при таймауте.
        """
        assert self.bus is not None
        msg = self.bus.recv(1.0)
        if msg is None:
            return -1
        if msg.arbitration_id == CAN_ID_HOST:
            return msg.data[0]
        return -1

    def _recv_bytes(self, count: int) -> List[int]:
        """Принять ровно count байт (каждый — отдельный CAN фрейм)."""
        result: List[int] = []
        timeout_count = 0
        while len(result) < count:
            byte = self._recv_byte()
            if byte < 0:
                timeout_count += 1
                if timeout_count > 20:
                    raise TimeoutError(f"Timeout waiting for byte {len(result)}/{count}")
                continue
            timeout_count = 0
            result.append(byte)
        return result

    def _validate_xor(self, data: bytes, received_xor: int) -> bool:
        """Проверить XOR контрольную сумму."""
        result = 0
        for b in data:
            result ^= b
        return result == received_xor

    # ─── Обработчики команд ──────────────────────────────────────────────

    def handle_sync(self) -> bool:
        """SYNC: ждём 0x7F, отвечаем ACK."""
        logger.info("SYNC")
        self._send_ack()
        return True

    def handle_cmd_get(self) -> bool:
        """CMD_GET (0x00). Ответ: N, version, commands..., ACK.

        Протокол:
          - хост шлёт CMD_GET + ~CMD_GET
          - устройство отвечает ACK
          - затем устройство шлёт N (bytes = число последующих байт - 1)
          - затем N+1 байт данных: version + [команды...]
          - затем ACK

        Здесь N должно быть равно len(cmd_list) (т.е. количеству команд),
        так как данных: version (1) + команды (N) = N+1.
        """
        logger.info("CMD_GET")
        self._send_ack()

        cmd_list = self.supported_commands
        n = len(cmd_list)  # количество команд = N
        # version (1) + команды (n) = n+1 байт данных

        self._send_byte(n)
        self._send_byte(self.version)
        for c in cmd_list:
            self._send_byte(c)
        self._send_byte(ACK_BYTE)

        logger.info(f"  Version=0x{self.version:02X}, Commands={[hex(c) for c in cmd_list]}")
        return True

    def handle_cmd_get_id(self) -> bool:
        """CMD_GET_ID (0x02). Ответ: N(=1), ID_hi, ID_lo, ACK."""
        logger.info("CMD_GET_ID")
        self._send_ack()

        n = 1  # 2 байта ID, N = 2-1 = 1
        id_hi = (self.chip_id >> 8) & 0xFF
        id_lo = self.chip_id & 0xFF
        self._send_byte(n)
        self._send_byte(id_hi)
        self._send_byte(id_lo)
        self._send_byte(ACK_BYTE)

        logger.info(f"  Chip ID=0x{self.chip_id:04X}")
        return True

    def handle_cmd_read_memory(self) -> bool:
        """CMD_READ_MEMORY (0x11). Чтение из виртуальной Flash."""
        logger.info("CMD_READ_MEMORY")
        self._send_ack()

        # Фаза 1: адрес (4 байта big-endian + XOR)
        addr_bytes = bytes(self._recv_bytes(4))
        addr_xor = self._recv_bytes(1)[0]
        if not self._validate_xor(addr_bytes, addr_xor):
            logger.warning("  Address XOR mismatch → NACK")
            self._send_nack()
            return True

        address = int.from_bytes(addr_bytes, 'big')
        offset = self._flash_to_offset(address)
        self._send_ack()

        # Фаза 2: N (размер - 1) + ~N
        n = self._recv_bytes(1)[0]
        n_complement = self._recv_bytes(1)[0]
        if n_complement != ((~n) & 0xFF):
            logger.warning("  N complement mismatch → NACK")
            self._send_nack()
            return True

        size = n + 1
        self._send_ack()

        # Чтение из flash
        if offset < 0 or offset + size > FLASH_SIZE:
            logger.warning(f"  Address out of range: 0x{address:08X}+{size}")
            read_data = bytes([0xFF] * size)
        else:
            read_data = bytes(self.flash[offset:offset + size])

        # Отправляем данные (1 байт = 1 фрейм)
        for b in read_data:
            self._send_byte(b)

        logger.info(f"  Read {size} bytes from 0x{address:08X}")
        return True

    def handle_cmd_write_memory(self) -> bool:
        """CMD_WRITE_MEMORY (0x31). Запись в виртуальную Flash."""
        logger.info("CMD_WRITE_MEMORY")
        self._send_ack()

        # Фаза 1: адрес
        addr_bytes = bytes(self._recv_bytes(4))
        addr_xor = self._recv_bytes(1)[0]
        if not self._validate_xor(addr_bytes, addr_xor):
            logger.warning("  Address XOR mismatch → NACK")
            self._send_nack()
            return True

        address = int.from_bytes(addr_bytes, 'big')
        offset = self._flash_to_offset(address)
        self._send_ack()

        # Фаза 2: N + data + XOR
        n = self._recv_bytes(1)[0]
        size = n + 1
        data_bytes = bytes(self._recv_bytes(size))
        xor_val = self._recv_bytes(1)[0]

        payload = bytes([n]) + data_bytes
        if not self._validate_xor(payload, xor_val):
            logger.warning("  Data XOR mismatch → NACK")
            self._send_nack()
            return True

        if offset < 0 or offset + size > FLASH_SIZE:
            logger.warning(f"  Address out of range: 0x{address:08X}+{size}")
            self._send_nack()
            return True

        self.flash[offset:offset + size] = data_bytes
        self._send_ack()

        logger.info(f"  Written {size} bytes to 0x{address:08X}")
        return True

    def handle_cmd_erase_memory(self) -> bool:
        """CMD_ERASE_MEMORY (0x43). Стирание Flash."""
        logger.info("CMD_ERASE_MEMORY")
        self._send_ack()

        # N + [pages...] + XOR  или  0xFF + ~0xFF (глобальное, 2 байта: без отдельного XOR)
        n = self._recv_bytes(1)[0]

        if n == 0xFF:
            # Глобальное стирание: 0xFF + ~0xFF (2 байта, без отдельного XOR)
            # 0x00 = ~0xFF — это одновременно complement и checksum для 0xFF
            byte2 = self._recv_bytes(1)[0]  # ~N = 0x00
            if byte2 != 0x00:
                logger.warning(f"  Global erase: expected 0x00, got 0x{byte2:02X} → NACK")
                self._send_nack()
                return True

            self.flash[:] = b'\xFF' * FLASH_SIZE
            self._send_ack()
            logger.info(f"  Global erase: {FLASH_SIZE} bytes")
        else:
            # Стирание по страницам
            pages_to_erase = n + 1
            pages = self._recv_bytes(pages_to_erase)
            xor_val = self._recv_bytes(1)[0]

            payload = bytes([n] + pages)
            if not self._validate_xor(payload, xor_val):
                logger.warning("  Page erase XOR mismatch → NACK")
                self._send_nack()
                return True

            self.flash[:] = b'\xFF' * FLASH_SIZE
            self._send_ack()
            logger.info(f"  Page erase: {pages_to_erase} pages")

        return True

    def handle_cmd_go(self) -> bool:
        """CMD_GO (0x21). Запуск приложения (остановка симулятора)."""
        logger.info("CMD_GO")
        self._send_ack()

        addr_bytes = bytes(self._recv_bytes(4))
        addr_xor = self._recv_bytes(1)[0]
        if not self._validate_xor(addr_bytes, addr_xor):
            logger.warning("  Address XOR mismatch → NACK")
            self._send_nack()
            return True

        address = int.from_bytes(addr_bytes, 'big')
        self._send_ack()

        logger.info(f"  *** Application started at 0x{address:08X} ***")
        print("\n" + "=" * 60)
        print(f"  🏁 GO command received for address 0x{address:08X}")
        print("  SIMULATOR: Application started! Stopping simulator.")
        print("=" * 60 + "\n")

        self.running = False
        return False

    # ─── Главный цикл ─────────────────────────────────────────────────────

    def run(self) -> None:
        """Запуск симулятора. Работает до получения CMD_GO или Ctrl+C."""
        print(f"\n{'=' * 60}")
        print(f"  CAN Bootloader Simulator (1 byte / frame)")
        print(f"  Channel: {self.channel} @ {self.bitrate} bps")
        print(f"  Chip ID: 0x{self.chip_id:04X} (GD32E230)")
        print(f"  Flash:   {FLASH_SIZE} bytes")
        print(f"{'=' * 60}")
        print(f"  Start GUI with: Interface=virtual, Channel={self.channel}")
        print(f"{'=' * 60}\n")

        self.bus = can.Bus(
            interface='virtual',
            channel=self.channel,
            bitrate=self.bitrate,
        )
        self.running = True

        try:
            while self.running:
                byte = self._recv_byte()
                if byte < 0:
                    continue

                # SYNC
                if byte == SYNC_BYTE:
                    self.handle_sync()
                    continue

                # Команды: cmd + ~cmd (2 байта)
                # Мы получили cmd, ждём complement
                complement_byte = self._recv_byte()
                while complement_byte < 0:
                    complement_byte = self._recv_byte()

                cmd = byte
                expected_comp = (~cmd) & 0xFF
                if complement_byte != expected_comp:
                    logger.warning(f"  Complement mismatch: 0x{complement_byte:02X} != "
                                   f"0x{expected_comp:02X} → NACK")
                    self._send_nack()
                    continue

                if cmd == CMD_GET:
                    self.handle_cmd_get()
                elif cmd == CMD_GET_ID:
                    self.handle_cmd_get_id()
                elif cmd == CMD_READ_MEMORY:
                    self.handle_cmd_read_memory()
                elif cmd == CMD_WRITE_MEMORY:
                    self.handle_cmd_write_memory()
                elif cmd == CMD_ERASE_MEMORY:
                    self.handle_cmd_erase_memory()
                elif cmd == CMD_GO:
                    if not self.handle_cmd_go():
                        break
                else:
                    logger.warning(f"  Unknown command: 0x{cmd:02X} → NACK")
                    self._send_nack()

        except KeyboardInterrupt:
            print("\nSimulator stopped by user")
        except Exception as e:
            logger.error(f"Error: {e}")
            raise
        finally:
            if self.bus:
                self.bus.shutdown()
            print("Simulator disconnected")


# ─── CLI Entry Point ────────────────────────────────────────────────────────

def main():
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(message)s")

    import argparse
    parser = argparse.ArgumentParser(description="CAN Bootloader Simulator")
    parser.add_argument("--channel", default="test_bus",
                        help="Virtual CAN channel (default: test_bus)")
    parser.add_argument("--bitrate", type=int, default=125000,
                        help="Bitrate (default: 125000)")
    parser.add_argument("--chip-id", type=lambda x: int(x, 16), default=0x0440,
                        help="Chip ID in hex (default: 0x0440 for GD32E230)")
    parser.add_argument("--version", type=lambda x: int(x, 16), default=0x22,
                        help="Bootloader version in hex (default: 0x22)")
    args = parser.parse_args()

    sim = CANBootloaderSimulator(
        channel=args.channel,
        bitrate=args.bitrate,
        chip_id=args.chip_id,
        version=args.version,
    )
    sim.run()


if __name__ == '__main__':
    main()
