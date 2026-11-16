#!/usr/bin/env python3
"""
CAN Flash Tool GUI — graphical interface for firmware flashing over CAN/UART.

Uses Azure ttk theme (https://github.com/rdbende/Azure-ttk-theme).

Supports:
  - CAN via candleLight USB-CAN adapter (python-can-candle)
  - UART via serial port (pyserial)

Dependencies: python-can, python-can-candle, intelhex, pyserial
"""

from __future__ import annotations

import logging
import os
import sys
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, simpledialog

from typing import Optional

import can
import serial

# Force candle interface registration for PyInstaller frozen builds
# (python-can-candle registers via entry points which PyInstaller doesn't see)
try:
    import candle  # noqa: F401
    import candle.candle_bus  # noqa: F401
    import candle_api  # noqa: F401 — native .pyd, required by candle.candle_bus
    if 'candle' not in can.interfaces.BACKENDS:
        can.interfaces.BACKENDS['candle'] = ('candle.candle_bus', 'CandleBus')
except ImportError:
    pass

from can_bootloader import (
    BaseBootloader,
    CANBootloader,
    BootloaderError,
    NackError,
    TimeoutError,
    ACK_BYTE,
    CAN_ID_HOST,
    CAN_ID_DEVICE,
    CMD_GET,
    CMD_GET_ID,
    CMD_READ_MEMORY,
    CMD_WRITE_MEMORY,
    CMD_ERASE_MEMORY,
    CMD_GO,
    CONNECTION_TIMEOUT,
    ChipInfo,
    lookup_chip,
)

from uart_bootloader import UARTBootloader

try:
    from can_bootloader_sim import CANBootloaderSimulator
    _SIM_AVAILABLE = True
except ImportError:
    _SIM_AVAILABLE = False

logger = logging.getLogger(__name__)

# Suppress noisy python-can log messages
logging.getLogger('can').setLevel(logging.ERROR)

# Maximum number of lines kept in the log widget. Bounds redraw and
# window-resize cost (a Text widget re-lays out every line on resize).
MAX_LOG_LINES = 2000

# ─── ToolTip helper ───────────────────────────────────────────────────────────

class _ToolTip:
    """Simple tooltip for tkinter widgets."""

    def __init__(self, widget: tk.Widget, text: str, delay_ms: int = 400):
        self.widget = widget
        self.text = text
        self.delay_ms = delay_ms
        self._tip_window: Optional[tk.Toplevel] = None
        self._after_id: Optional[str] = None
        widget.bind('<Enter>', self._schedule, add='+')
        widget.bind('<Leave>', self._hide, add='+')

    def _schedule(self, event=None):
        self._after_id = self.widget.after(self.delay_ms, self._show)

    def _show(self):
        if self._tip_window:
            return
        x = self.widget.winfo_rootx() + 20
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 5
        self._tip_window = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        label = tk.Label(tw, text=self.text, justify=tk.LEFT,
                         background="#ffffe0", foreground="#000000",
                         relief=tk.SOLID, borderwidth=1,
                         font=("Consolas", 8))
        label.pack()

    def _hide(self, event=None):
        if self._after_id:
            self.widget.after_cancel(self._after_id)
            self._after_id = None
        if self._tip_window:
            self._tip_window.destroy()
            self._tip_window = None


# ─── Helper functions ─────────────────────────────────────────────────────────

def _format_size(size: int) -> str:
    """Format a byte size to a human-readable string."""
    if size < 1024:
        return f"{size} B"
    elif size < 1024 * 1024:
        return f"{size / 1024:.1f} KB"
    else:
        return f"{size / 1024 / 1024:.2f} MB"


def _validate_hex_limiter(inserted: str, max_pairs_str: str) -> bool:
    """
    ``validate='key'`` callback for Entry.
    Returns False (rejects input) if the number of hex characters in the
    resulting text already exceeds the limit — this prevents ANY flicker
    because Tkinter never inserts the character at all.

    ``max_pairs_str`` is passed as a string from Tcl, so convert to int.
    """
    max_hex = int(max_pairs_str) * 2
    if max_hex <= 0:
        return True
    cleaned = ''.join(c for c in inserted if c in '0123456789abcdefABCDEF')
    return len(cleaned) <= max_hex


def _auto_format_hex(entry: tk.Entry, max_pairs: int = 0,
                     counter_label: Optional[ttk.Label] = None, *_):
    """
    KeyRelease callback on an Entry widget that auto-formats hex input into
    space-separated byte pairs.  E.g. typing ``55aaff`` → ``55 aa ff``.

    Does NOT truncate partial (odd-length) input, so the user can type
    character by character.  Saves and restores the cursor position so the
    cursor does not jump to the end after formatting.

    If ``max_pairs > 0`` limits to that many bytes (enforced by
    ``validate='key'``, not here — here we only format and count).
    If ``counter_label`` is provided, updates its text to ``N/max_pairs``.
    """
    raw = entry.get()
    # cursor position BEFORE we mess with the text
    try:
        cursor_raw = entry.index(tk.INSERT)
    except Exception:
        cursor_raw = len(raw)

    # Remove all non-hex characters
    cleaned = ''.join(c for c in raw if c in '0123456789abcdefABCDEF')

    # ── Update byte counter ────────────────────────────────────────────
    if counter_label is not None and max_pairs > 0:
        # Count complete bytes (partial odd hex char → +1 byte)
        hex_len = len(cleaned)
        bytes_cnt = (hex_len + 1) // 2  # ceil(hex_len / 2)
        if bytes_cnt > max_pairs:
            bytes_cnt = max_pairs
        counter_label.config(text=f"{bytes_cnt}/{max_pairs}")

    # Split into pairs, keep trailing odd char if any
    pairs = []
    i = 0
    while i + 1 <= len(cleaned):
        pairs.append(cleaned[i:i+2])
        i += 2

    formatted = ' '.join(pairs)

    if raw == formatted:
        return

    # ── Figure out where the cursor should go ──────────────────────────
    # Count how many *hex characters* are before the cursor in the raw text
    hex_before = 0
    for c in raw[:cursor_raw]:
        if c in '0123456789abcdefABCDEF':
            hex_before += 1

    hex_before = min(hex_before, len(cleaned))

    new_pos = 0
    seen = 0
    for i, ch in enumerate(formatted):
        if seen >= hex_before:
            new_pos = i
            break
        if ch in '0123456789abcdefABCDEF':
            seen += 1
        new_pos = i + 1

    # ── Apply ──────────────────────────────────────────────────────────
    entry.delete(0, tk.END)
    entry.insert(0, formatted)
    entry.icursor(new_pos)


def _parse_hex_payload(text: str) -> Optional[bytes]:
    """
    Parse hex payload string (space-separated hex bytes, no 0x prefix).

    Returns bytes or None on error.
    """
    text = text.strip()
    if not text:
        return b''
    try:
        # Remove commas, replace with spaces, then split
        clean = text.replace(',', ' ')
        parts = clean.split()
        return bytes(int(p, 16) for p in parts)
    except (ValueError, IndexError):
        return None


# ─── GUI Application ──────────────────────────────────────────────────────────


class CANFlashToolGUI:
    """Main application window."""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("CAN Flash Tool")
        self.root.minsize(700, 550)

        # State
        self.bootloader: Optional[BaseBootloader] = None
        self.connected = False
        self.hex_path: Optional[str] = None
        self.hex_data = None  # IntelHex object
        self.hex_segments: list = []
        self.hex_total_size = 0
        self._running_op = False
        self._connect_timer: Optional[str] = None
        self._simulator: Optional[CANBootloaderSimulator] = None
        self._simulator_thread: Optional[threading.Thread] = None
        self._simulator_running = False

        # Ping monitor state
        self._last_activity_time = 0.0
        self._ping_monitor_timer: Optional[str] = None
        self._ping_pending = False

        # Interface availability cache
        self._has_com_ports = False
        self._has_candle_devices = False

        # Determine resource path (for PyInstaller --onefile support)
        if hasattr(sys, '_MEIPASS'):
            self._res_path = sys._MEIPASS
        else:
            self._res_path = os.path.dirname(os.path.abspath(__file__))

        # Style setup
        self._setup_styles()

        # Build UI
        self._build_ui()

        # Size the window to fit the built content. Text/font metrics can be
        # larger on scaled/high-DPI displays, so clamp to a comfortable minimum
        # instead of a hardcoded "800x650" set before the widgets existed.
        self.root.update_idletasks()
        win_w = max(800, self.root.winfo_reqwidth())
        win_h = max(650, self.root.winfo_reqheight())
        self.root.geometry(f"{win_w}x{win_h}")

        # Close handler
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _setup_styles(self):
        """Configure ttk styles with Azure dark theme."""
        azure_tcl = os.path.join(self._res_path, "azure.tcl")
        self.root.tk.call("source", azure_tcl)
        self.root.tk.call("set_theme", "dark")

        style = ttk.Style()
        style.configure("Highlight.TLabel", foreground="#569cd6",
                        font=("TkFixedFont", 10, "bold"))
        style.configure("Short.TEntry", padding=(3, 1))

        self.root.option_add('*TCombobox*Listbox.background', '#333333')
        self.root.option_add('*TCombobox*Listbox.foreground', '#ffffff')
        self.root.option_add('*TCombobox*Listbox.selectBackground', "#529ae2")
        self.root.option_add('*TCombobox*Listbox.selectForeground', '#ffffff')

    def _build_ui(self):
        """Build the user interface."""
        # ─── Connection Panel ────────────────────────────────────────────────
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=(10, 5))

        # Row 0: Interface / Channel / Scan / Baudrate / Parity / Connect
        ttk.Label(conn_frame, text="Interface:").grid(row=0, column=0, padx=5, pady=3, sticky=tk.W)

        self.interface_var = tk.StringVar(value="")
        self.interface_combo = ttk.Combobox(conn_frame, textvariable=self.interface_var,
                                            values=["virtual"],
                                            state="readonly", width=10)
        self.interface_combo.grid(row=0, column=1, padx=3, pady=3, sticky=tk.W)
        self.interface_var.trace_add("write", self._on_interface_change)

        ttk.Label(conn_frame, text="Channel:").grid(row=0, column=2, padx=3, pady=3, sticky=tk.W)
        self.channel_var = tk.StringVar(value="")
        self.channel_combo = ttk.Combobox(conn_frame, textvariable=self.channel_var,
                                           values=[], width=12, state="readonly")
        self.channel_combo.grid(row=0, column=3, padx=3, pady=3, sticky=tk.W)
        self.scan_btn = ttk.Button(conn_frame, text="Scan", command=self._scan_devices, width=5)
        self.scan_btn.grid(row=0, column=4, padx=(1, 3), pady=3)

        self.bitrate_label = ttk.Label(conn_frame, text="Baudrate:")
        self.bitrate_label.grid(row=0, column=5, padx=3, pady=3, sticky=tk.W)
        self.bitrate_var = tk.StringVar(value="115200")
        self.bitrate_combo = ttk.Combobox(conn_frame, textvariable=self.bitrate_var,
                                          values=["125000", "250000", "500000", "1000000"],
                                          state="readonly", width=9)
        self.bitrate_combo.grid(row=0, column=6, padx=3, pady=3, sticky=tk.W)

        # Parity combo (visible only for UART)
        self._parity_label = ttk.Label(conn_frame, text="Parity:")
        self._parity_label.grid(row=0, column=7, padx=3, pady=3, sticky=tk.W)
        self.uart_parity_var = tk.StringVar(value="NONE")
        self._parity_combo = ttk.Combobox(conn_frame, textvariable=self.uart_parity_var,
                                          values=["NONE", "ODD", "EVEN"],
                                          state="readonly", width=6)
        self._parity_combo.grid(row=0, column=8, padx=3, pady=3, sticky=tk.W)

        self.connect_btn = ttk.Button(conn_frame, text="Connect",
                                      command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=9, padx=(10, 5), pady=3)

        # Row 1: hint label
        self.hint_var = tk.StringVar(value="")
        self.hint_label = ttk.Label(conn_frame, textvariable=self.hint_var,
                                    foreground="#dcdcaa", font=("Consolas", 8))
        self.hint_label.grid(row=1, column=0, columnspan=10, padx=5, pady=(0, 2), sticky=tk.W)

        # ─── Pre-handshake message frame (row 2) ────────────────────────────
        pre_frame = ttk.Frame(conn_frame)
        pre_frame.grid(row=2, column=0, columnspan=10, sticky=tk.W, padx=5, pady=(2, 0))

        self.pre_enable_var = tk.BooleanVar(value=False)
        self.pre_checkbtn = ttk.Checkbutton(
            pre_frame, text="Send pre-handshake msg",
            variable=self.pre_enable_var,
            command=self._on_pre_enable_change
        )
        self.pre_checkbtn.pack(side=tk.LEFT, padx=(0, 10))
        _ToolTip(self.pre_checkbtn, "Useful for force device jump to bootloader")

        # ─── CAN pre-handshake fields ──────────────────────────────────────
        #   CAN ID | PAYLOAD (hex) | auto-DLC from byte count
        self._pre_can_frame = ttk.Frame(pre_frame)
        self._pre_can_frame.pack(side=tk.LEFT)

        ttk.Label(self._pre_can_frame, text="CAN ID:").pack(side=tk.LEFT, padx=(0, 3))
        self.pre_id_var = tk.StringVar(value="0x7FF")
        self.pre_id_entry = ttk.Entry(self._pre_can_frame, textvariable=self.pre_id_var, width=8)
        self.pre_id_entry.pack(side=tk.LEFT, padx=(0, 10))

        ttk.Label(self._pre_can_frame, text="PAYLOAD (hex):").pack(side=tk.LEFT, padx=(0, 3))
        self.pre_payload_var = tk.StringVar(value="FF")
        self.pre_payload_entry = ttk.Entry(self._pre_can_frame, textvariable=self.pre_payload_var, width=30)
        self.pre_payload_entry.pack(side=tk.LEFT, padx=(0, 3))
        # validate='key' — block input if > 16 hex characters (8 bytes), NO flicker
        self._validate_can = self.root.register(_validate_hex_limiter)
        self.pre_payload_entry.config(
            validate='key',
            validatecommand=(self._validate_can, '%P', '8')
        )
        # Counter label (e.g. "0/8", "2/8", "8/8")
        self._can_payload_counter = ttk.Label(self._pre_can_frame, text="0/8",
                                                foreground="#808080", font=("Consolas", 8))
        self._can_payload_counter.pack(side=tk.LEFT, padx=(0, 5))
        # Auto-format on KeyRelease
        self.pre_payload_entry.bind(
            '<KeyRelease>',
            lambda e, en=self.pre_payload_entry, cl=self._can_payload_counter:
                _auto_format_hex(en, max_pairs=8, counter_label=cl)
        )

        # ─── UART pre-handshake fields ─────────────────────────────────────
        # Two rows (pack-aligned):
        #   Row 0: "UART payload (hex):" [entry] counter
        #   Row 1 (indented): "Interval [ms]:" [entry] "Pre-handshake baud:" [combo]
        self._pre_uart_frame = ttk.Frame(pre_frame)

        # Row 0 — payload label + entry + counter
        pre_uart_row0 = ttk.Frame(self._pre_uart_frame)
        pre_uart_row0.pack(anchor=tk.W, pady=(0, 2))

        ttk.Label(pre_uart_row0, text="UART payload (hex):").pack(side=tk.LEFT, padx=(0, 3))
        self.pre_uart_payload_var = tk.StringVar(value="")
        self.pre_uart_payload_entry = ttk.Entry(pre_uart_row0,
                                                 textvariable=self.pre_uart_payload_var,
                                                 width=29, style="Short.TEntry")
        self.pre_uart_payload_entry.pack(side=tk.LEFT, padx=(0, 3))
        # validate='key' — block input if > 2048 hex characters (1024 bytes), NO flicker
        self._validate_uart = self.root.register(_validate_hex_limiter)
        self.pre_uart_payload_entry.config(
            validate='key',
            validatecommand=(self._validate_uart, '%P', '1024')
        )
        # Counter label (e.g. "0/1024", "512/1024")
        self._uart_payload_counter = ttk.Label(pre_uart_row0, text="0/1024",
                                                foreground="#808080", font=("Consolas", 8))
        self._uart_payload_counter.pack(side=tk.LEFT, padx=(0, 2))
        # Auto-format on KeyRelease
        self.pre_uart_payload_entry.bind(
            '<KeyRelease>',
            lambda e, en=self.pre_uart_payload_entry, cl=self._uart_payload_counter:
                _auto_format_hex(en, max_pairs=1024, counter_label=cl)
        )

        # Row 1 — interval + baud (indented to align interval entry with payload entry)
        pre_uart_row1 = ttk.Frame(self._pre_uart_frame)
        pre_uart_row1.pack(anchor=tk.W, padx=(40, 0))

        ttk.Label(pre_uart_row1, text="Interval [ms]:").pack(side=tk.LEFT, padx=(0, 3))
        self.pre_uart_interval_var = tk.StringVar(value="10")
        self.pre_uart_interval_entry = ttk.Entry(pre_uart_row1,
                                                  textvariable=self.pre_uart_interval_var,
                                                  width=5, style="Short.TEntry")
        self.pre_uart_interval_entry.pack(side=tk.LEFT, padx=(3, 5))

        ttk.Label(pre_uart_row1, text="Pre-handshake baud:").pack(side=tk.LEFT, padx=(0, 3))
        self.pre_uart_baud_var = tk.StringVar(value="9600")
        self.pre_uart_baud_combo = ttk.Combobox(
            pre_uart_row1, textvariable=self.pre_uart_baud_var,
            values=["9600", "19200", "38400", "57600", "115200",
                    "230400", "460800", "921600", "1000000", "2000000"],
            state="readonly", width=10
        )
        self.pre_uart_baud_combo.pack(side=tk.LEFT, padx=(0, 3))

        # Initially hide pre-handshake fields
        self._set_can_pre_fields_visible(False)
        self._set_uart_pre_fields_visible(False)

        # ─── Device Info Panel ────────────────────────────────────────────────
        info_frame = ttk.LabelFrame(self.root, text="Device Info", padding=10)
        info_frame.pack(fill=tk.X, padx=10, pady=5)

        self.info_vars = {
            "version": tk.StringVar(value="--"),
            "chip_id": tk.StringVar(value="--"),
            "chip_name": tk.StringVar(value="--"),
            "commands": tk.StringVar(value="--"),
            "flash_page": tk.StringVar(value="--"),
            "flash_size": tk.StringVar(value="--"),
        }

        # Row 0
        ttk.Label(info_frame, text="Bootloader Version:").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        ttk.Label(info_frame, textvariable=self.info_vars["version"],
                  style="Highlight.TLabel").grid(row=0, column=1, padx=5, pady=2, sticky=tk.W)
        ttk.Label(info_frame, text="Chip ID:").grid(row=0, column=2, padx=(20, 5), pady=2, sticky=tk.W)
        ttk.Label(info_frame, textvariable=self.info_vars["chip_id"],
                  style="Highlight.TLabel").grid(row=0, column=3, padx=5, pady=2, sticky=tk.W)
        ttk.Label(info_frame, text="MCU:").grid(row=0, column=4, padx=(20, 5), pady=2, sticky=tk.W)
        ttk.Label(info_frame, textvariable=self.info_vars["chip_name"],
                  style="Highlight.TLabel").grid(row=0, column=5, padx=5, pady=2, sticky=tk.W)

        # Row 1
        ttk.Label(info_frame, text="Commands:").grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        ttk.Label(info_frame, textvariable=self.info_vars["commands"],
                  style="Highlight.TLabel").grid(row=1, column=1, padx=5, pady=2, sticky=tk.W)
        ttk.Label(info_frame, text="Flash Page:").grid(row=1, column=2, padx=(20, 5), pady=2, sticky=tk.W)
        ttk.Label(info_frame, textvariable=self.info_vars["flash_page"],
                  style="Highlight.TLabel").grid(row=1, column=3, padx=5, pady=2, sticky=tk.W)
        ttk.Label(info_frame, text="Flash Size:").grid(row=1, column=4, padx=(20, 5), pady=2, sticky=tk.W)
        ttk.Label(info_frame, textvariable=self.info_vars["flash_size"],
                  style="Highlight.TLabel").grid(row=1, column=5, padx=5, pady=2, sticky=tk.W)

        # ─── Flash Operations Panel ────────────────────────────────────────────
        ops_frame = ttk.LabelFrame(self.root, text="Flash Operations", padding=10)
        ops_frame.pack(fill=tk.BOTH, padx=10, pady=5, expand=True)

        # Hex file selection
        hex_row = ttk.Frame(ops_frame)
        hex_row.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(hex_row, text="Hex File:").pack(side=tk.LEFT, padx=(0, 5))
        self.hex_path_var = tk.StringVar(value="")
        self.hex_entry = ttk.Entry(hex_row, textvariable=self.hex_path_var, state="readonly")
        self.hex_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self.browse_btn = ttk.Button(hex_row, text="Browse...", command=self._browse_hex)
        self.browse_btn.pack(side=tk.LEFT)

        # Start Addr field + Erase/Verify checkboxes
        addr_frame = ttk.Frame(ops_frame)
        addr_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(addr_frame, text="Start Addr:").pack(side=tk.LEFT, padx=(0, 5))
        self.start_addr_var = tk.StringVar(value="0x08000800")
        self.start_addr_entry = ttk.Entry(addr_frame, textvariable=self.start_addr_var, width=14)
        self.start_addr_entry.pack(side=tk.LEFT, padx=(0, 15))

        self._prog_erase_var = tk.BooleanVar(value=True)
        self._prog_verify_var = tk.BooleanVar(value=True)
        self._prog_start_var = tk.BooleanVar(value=False)

        ttk.Checkbutton(addr_frame, text="Erase flash before program",
                        variable=self._prog_erase_var).pack(side=tk.LEFT, padx=(0, 15))
        ttk.Checkbutton(addr_frame, text="Verify after program",
                        variable=self._prog_verify_var).pack(side=tk.LEFT)

        # Action buttons
        btn_frame = ttk.Frame(ops_frame)
        btn_frame.pack(fill=tk.X, pady=(0, 10))

        self.erase_btn = ttk.Button(btn_frame, text="Erase", command=self._erase_flash, width=12)
        self.erase_btn.pack(side=tk.LEFT, padx=(0, 5))
        self.program_btn = ttk.Button(btn_frame, text="Program", command=self._program_flash, width=12)
        self.program_btn.pack(side=tk.LEFT, padx=(0, 5))
        self.read_btn = ttk.Button(btn_frame, text="Read", command=self._read_memory, width=12)
        self.read_btn.pack(side=tk.LEFT, padx=(0, 5))
        self.verify_btn = ttk.Button(btn_frame, text="Verify", command=self._verify_flash, width=12)
        self.verify_btn.pack(side=tk.LEFT, padx=(0, 5))
        self.go_btn = ttk.Button(btn_frame, text="Go", command=self._go_cmd, width=12)
        self.go_btn.pack(side=tk.LEFT, padx=(0, 5))

        ttk.Checkbutton(btn_frame, text="Start after program",
                        variable=self._prog_start_var).pack(side=tk.LEFT, padx=(20, 0))

        # Progress bar
        self.progress_var = tk.IntVar(value=0)
        self.progress_bar = ttk.Progressbar(ops_frame, variable=self.progress_var, maximum=100)
        self.progress_bar.pack(fill=tk.X, pady=(0, 5))

        self.status_label = ttk.Label(ops_frame, text="Ready")
        self.status_label.pack(anchor=tk.W)

        # ─── Log Panel ────────────────────────────────────────────────────────
        log_frame = ttk.LabelFrame(self.root, text="Log", padding=10)
        log_frame.pack(fill=tk.BOTH, padx=10, pady=(5, 10), expand=True)

        log_text_frame = ttk.Frame(log_frame)
        log_text_frame.pack(fill=tk.BOTH, expand=True)

        # wrap=NONE avoids re-wrapping the whole log on every window resize,
        # which is the main cause of resize lag with a growing Text widget.
        self.log_text = tk.Text(log_text_frame, height=10, wrap=tk.NONE,
                                bg="#1e1e1e", fg="#d4d4d4",
                                insertbackground="white",
                                font=("Consolas", 9))
        self.log_text.grid(row=0, column=0, sticky="nsew")

        log_scroll = ttk.Scrollbar(log_text_frame, orient=tk.VERTICAL,
                                   command=self.log_text.yview)
        log_scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.config(yscrollcommand=log_scroll.set)

        log_hscroll = ttk.Scrollbar(log_text_frame, orient=tk.HORIZONTAL,
                                    command=self.log_text.xview)
        log_hscroll.grid(row=1, column=0, sticky="ew")
        self.log_text.config(xscrollcommand=log_hscroll.set)

        log_text_frame.rowconfigure(0, weight=1)
        log_text_frame.columnconfigure(0, weight=1)

        self.log_text.tag_configure("info", foreground="#d4d4d4")
        self.log_text.tag_configure("ok", foreground="#4ec9b0")
        self.log_text.tag_configure("error", foreground="#f44747")
        self.log_text.tag_configure("warn", foreground="#dcdcaa")

        self.log("CAN Flash Tool v1.1 ready", "info")

        # ─── Initial button states ────────────────────────────────────────────
        self._set_controls_state(connected=False)

        # Auto-detect available interfaces at startup (ONCE)
        self.log("Detecting available interfaces...", "info")
        self._detect_interfaces()

    # ─── Pre-handshake helpers ─────────────────────────────────────────────────

    def _on_pre_enable_change(self):
        """Show/hide pre-handshake fields based on checkbox state and interface."""
        visible = self.pre_enable_var.get()
        interface = self.interface_var.get()
        if interface == "uart":
            self._set_can_pre_fields_visible(False)
            self._set_uart_pre_fields_visible(visible)
        else:
            self._set_uart_pre_fields_visible(False)
            self._set_can_pre_fields_visible(visible)

    def _set_can_pre_fields_visible(self, visible: bool):
        if visible:
            self._pre_can_frame.pack(side=tk.LEFT)
        else:
            self._pre_can_frame.pack_forget()

    def _set_uart_pre_fields_visible(self, visible: bool):
        if visible:
            self._pre_uart_frame.pack(side=tk.LEFT, padx=(0, 5))
        else:
            self._pre_uart_frame.pack_forget()

    def _send_uart_prehandshake_and_rebaud(self, bl: UARTBootloader,
                                            pre_baud: int,
                                            main_baud: int) -> bool:
        """
        Perform UART pre-handshake using the given UARTBootloader instance.

        Opens serial at pre_baud, sends payload bytes with interval,
        then switches baudrate to main_baud.

        Returns True on success, False on failure.
        """
        import time

        # Parse payload
        payload_str = self.pre_uart_payload_var.get().strip()
        if not payload_str:
            self.root.after(0, lambda: self.log("UART pre-handshake: empty payload — skipping", "warn"))
            return True

        payload = _parse_hex_payload(payload_str)
        if payload is None:
            self.root.after(0, lambda: self.log(
                f"UART pre-handshake: invalid hex payload '{payload_str}'", "error"))
            return False

        if len(payload) > 1024:
            self.root.after(0, lambda: self.log(
                f"UART pre-handshake: payload too long ({len(payload)} > 1024)", "error"))
            return False

        try:
            interval = float(self.pre_uart_interval_var.get().strip())
        except ValueError:
            self.root.after(0, lambda: self.log(
                f"UART pre-handshake: invalid Interval '{self.pre_uart_interval_var.get()}'", "error"))
            return False

        # Open serial at pre-handshake baud (overrides bootloader's default)
        self.root.after(0, lambda: self.log(
            f"UART pre-handshake: opening {bl.port} at {pre_baud} baud...", "info"))
        try:
            bl.connect()  # opens at bootloader.baudrate (= pre_baud)
        except Exception as e:
            self.root.after(0, lambda msg=str(e): self.log(
                f"UART pre-handshake: cannot open port: {msg}", "error"))
            return False

        # Send payload bytes with interval
        self.root.after(0, lambda cnt=len(payload): self.log(
            f"UART pre-handshake: sending {cnt} byte(s)...", "info"))
        try:
            for i, b in enumerate(payload):
                bl._send_byte(b)
                if interval > 0 and i < len(payload) - 1:
                    time.sleep(interval / 1000.0)
        except Exception as e:
            self.root.after(0, lambda msg=str(e): self.log(
                f"UART pre-handshake: send error: {msg}", "error"))
            try:
                bl.disconnect()
            except Exception:
                pass
            return False

        # Switch to main baudrate
        self.root.after(0, lambda: self.log(
            f"UART pre-handshake: switching to {main_baud} baud...", "info"))
        try:
            bl.set_baudrate(main_baud)
            # Flush stale RX bytes left from pre-handshake baud to avoid garbage reads
            bl.ser.reset_input_buffer()
        except Exception as e:
            self.root.after(0, lambda msg=str(e): self.log(
                f"UART pre-handshake: baudrate change failed: {msg}", "error"))
            try:
                bl.disconnect()
            except Exception:
                pass
            return False

        self.root.after(0, lambda: self.log("UART pre-handshake: complete", "info"))
        return True

    def _send_can_prehandshake(self, bootloader: CANBootloader) -> bool:
        """
        Send the CAN pre-handshake message on the bootloader's bus.
        DLC is auto-calculated from the number of bytes in payload.

        Returns True on success, False on failure.
        """
        if not self.pre_enable_var.get():
            return True

        # Parse CAN ID
        try:
            can_id_str = self.pre_id_var.get().strip()
            if can_id_str.startswith("0x") or can_id_str.startswith("0X"):
                can_id = int(can_id_str, 16)
            else:
                can_id = int(can_id_str)
        except ValueError:
            self.root.after(0, lambda: self.log(
                f"Pre-handshake: invalid CAN ID '{self.pre_id_var.get()}'", "error"))
            return False

        # Parse payload (hex bytes, no 0x prefix needed)
        payload_bytes = _parse_hex_payload(self.pre_payload_var.get())
        if payload_bytes is None:
            self.root.after(0, lambda: self.log(
                f"Pre-handshake: invalid PAYLOAD hex string '{self.pre_payload_var.get()}'", "error"))
            return False

        if len(payload_bytes) > 8:
            self.root.after(0, lambda: self.log(
                f"Pre-handshake: CAN DLC max 8 bytes, got {len(payload_bytes)}", "error"))
            return False

        dlc = len(payload_bytes)

        # Skip if both ID and DLC are zero
        if can_id == 0 and dlc == 0:
            self.root.after(0, lambda: self.log(
                "Pre-handshake: CAN ID=0 and payload empty — skipping", "warn"))
            return True

        # Send the message
        try:
            msg = can.Message(
                arbitration_id=can_id,
                data=payload_bytes,
                is_extended_id=False,
                dlc=dlc,
            )
            bootloader.bus.send(msg)
            self.root.after(0, lambda: self.log(
                f"Pre-handshake: sent ID=0x{can_id:X}, DLC={dlc}, "
                f"data={payload_bytes.hex(' ')}", "info"))
        except Exception as e:
            self.root.after(0, lambda msg=str(e): self.log(
                f"Pre-handshake send error: {msg}", "error"))
            return False

        return True

    # ─── Interface detection (ONCE at startup) ────────────────────────────────

    def _detect_interfaces(self):
        """Detect which interface types are available (candle, slcan, uart).

        Called once at startup. Scan button only refreshes channel list for
        the currently selected interface.
        """
        def do_detect():
            available = []

            if self._simulator_running:
                available.append("virtual")

            # Check for candleLight USB-CAN devices
            try:
                configs = can.detect_available_configs('candle')
                if configs:
                    available.append("candle")
                    self._has_candle_devices = True
                    num = len(configs)
                    self.root.after(0, lambda n=num: self.log(
                        f"  Found {n} candleLight USB-CAN adapter(s)", "ok"))
            except Exception:
                pass

            # Check for COM ports (slcan + uart)
            try:
                import serial.tools.list_ports
                ports = list(serial.tools.list_ports.comports())
                if ports:
                    available.append("slcan")
                    available.append("uart")
                    self._has_com_ports = True
                    self.root.after(0, lambda p=ports: self.log(
                        f"  Found COM ports: {', '.join(p.device for p in ports)}", "ok"))
            except ImportError:
                pass
            except Exception:
                pass

            self.root.after(0, lambda a=available: self._update_interfaces(a))

        threading.Thread(target=do_detect, daemon=True).start()

    def _update_interfaces(self, available: list):
        """Update interface dropdown and auto-select best one."""
        self.interface_combo.config(values=available)
        if available:
            preferred = "candle" if self._has_candle_devices else (
                "uart" if self._has_com_ports else (
                    "slcan" if self._has_com_ports else available[0]))
            if preferred in available:
                self.interface_var.set(preferred)
            else:
                self.interface_var.set(available[0])
            self._on_interface_change()

    # ─── Interface change handler ──────────────────────────────────────────────

    def _on_interface_change(self, *_):
        """Adjust UI elements based on selected interface type + trigger auto-scan."""
        interface = self.interface_var.get()
        hints = {
            "candle": "USB-CAN via candleLight firmware (python-can-candle). Press Scan to list devices.",
            "slcan": "Serial CAN over COM port. Press Scan to list ports, then select from list.",
            "virtual": "Virtual bus for testing (requires --sim flag).",
            "uart": "UART serial bootloader via USB-UART adapter. Select COM port from list.",
        }
        self.hint_var.set(hints.get(interface, ""))

        # Toggle pre-handshake field visibility + parity
        if interface == "uart":
            self._set_can_pre_fields_visible(False)
            self._set_uart_pre_fields_visible(self.pre_enable_var.get())
            self.bitrate_label.config(text="Baudrate:")
            uart_rates = ["9600", "19200", "38400", "57600", "115200",
                          "230400", "460800", "921600", "1000000", "2000000"]
            self.bitrate_combo.config(values=uart_rates)
            if self.bitrate_var.get() not in uart_rates:
                self.bitrate_var.set("115200")
            self._parity_label.grid()
            self._parity_combo.grid()
        else:
            self._set_uart_pre_fields_visible(False)
            self._set_can_pre_fields_visible(True)
            self.bitrate_label.config(text="Bitrate:")
            can_rates = ["125000", "250000", "500000", "1000000"]
            self.bitrate_combo.config(values=can_rates)
            if self.bitrate_var.get() not in can_rates:
                self.bitrate_var.set("125000")
            self._parity_label.grid_remove()
            self._parity_combo.grid_remove()

        # Auto-scan for the selected interface
        self.root.after(100, self._scan_devices)

    # ─── Logging ───────────────────────────────────────────────────────────────

    def log(self, message: str, tag: str = "info"):
        """Add a message to the log panel (keeps the last MAX_LOG_LINES lines)."""
        self.log_text.insert(tk.END, f"{message}\n", tag)
        # Trim oldest lines so the widget stays bounded (redraw/resize cost).
        # 'end-1c' lands on Tk's trailing empty line, so subtract it (-1).
        line_count = int(self.log_text.index("end-1c").split(".")[0]) - 1
        if line_count > MAX_LOG_LINES:
            self.log_text.delete("1.0", f"{line_count - MAX_LOG_LINES + 1}.0")
        self.log_text.see(tk.END)

    # ─── State management ──────────────────────────────────────────────────────

    def _set_controls_state(self, connected: bool):
        """Enable or disable controls based on connection state."""
        op_state = "normal" if connected else "disabled"
        for btn in [self.erase_btn, self.program_btn, self.read_btn,
                    self.verify_btn, self.go_btn]:
            btn.config(state=op_state)

        if connected:
            self.connect_btn.config(text="Disconnect", state="normal")
        else:
            self.connect_btn.config(text="Connect", state="normal")

    def _on_op_start(self):
        """Lock all buttons while an operation is in progress."""
        self._running_op = True
        self.erase_btn.config(state="disabled")
        self.program_btn.config(state="disabled")
        self.read_btn.config(state="disabled")
        self.verify_btn.config(state="disabled")
        self.go_btn.config(state="disabled")
        self.connect_btn.config(state="disabled")
        self.browse_btn.config(state="disabled")

    def _on_op_end(self):
        """Unlock buttons after an operation completes."""
        self._running_op = False
        if self.connected:
            self._set_controls_state(connected=True)
        else:
            self._set_controls_state(connected=False)
        self.connect_btn.config(state="normal")
        self.browse_btn.config(state="normal")

    def _update_progress(self, value: int, status: str):
        """Update progress bar and status text (thread-safe).

        Always called on the main thread via root.after(0, ...), so the event
        loop redraws on its own; an explicit update_idletasks() only adds
        synchronous redraw overhead during long operations.
        """
        self.progress_var.set(value)
        self.status_label.config(text=status)

    # ─── Ping monitor ──────────────────────────────────────────────────────────

    def _mark_activity(self):
        import time
        self._last_activity_time = time.monotonic()

    def _start_ping_monitor(self):
        self._stop_ping_monitor()
        self._mark_activity()
        self._ping_monitor_timer = self.root.after(3000, self._ping_monitor_check)

    def _stop_ping_monitor(self):
        if self._ping_monitor_timer is not None:
            self.root.after_cancel(self._ping_monitor_timer)
            self._ping_monitor_timer = None
        self._ping_pending = False

    def _ping_monitor_check(self):
        import time
        self._ping_monitor_timer = None

        if not self.connected or self._running_op or self._ping_pending:
            if self.connected:
                self._ping_monitor_timer = self.root.after(3000, self._ping_monitor_check)
            return

        elapsed = time.monotonic() - self._last_activity_time
        if elapsed < 3.0:
            self._ping_monitor_timer = self.root.after(3000, self._ping_monitor_check)
            return

        self._ping_pending = True

        def do_ping():
            try:
                alive = self.bootloader.ping(timeout=0.5)
            except Exception:
                alive = False
            self.root.after(0, lambda: self._ping_result(alive))

        threading.Thread(target=do_ping, daemon=True).start()

    def _ping_result(self, alive: bool):
        self._ping_pending = False

        if alive:
            self._mark_activity()
            if self.connected:
                self._ping_monitor_timer = self.root.after(3000, self._ping_monitor_check)
        else:
            self.log("Ping: device not responding — disconnecting", "error")
            self._stop_ping_monitor()
            if self.connected:
                self.connected = False
                if self.bootloader:
                    try:
                        self.bootloader.disconnect()
                    except Exception:
                        pass
                    self.bootloader = None
                self._set_controls_state(connected=False)
                for key in self.info_vars:
                    self.info_vars[key].set("--")
                self.progress_var.set(0)
                self.status_label.config(text="Disconnected (ping failed)")
                self.log("Disconnected due to ping timeout", "warn")

    # ─── Button handlers ───────────────────────────────────────────────────────

    def _toggle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect_timeout(self):
        self._connect_timer = None
        if not self.connected and self.bootloader is not None:
            self.log("Connection timeout: no response from device", "error")
            self._update_progress(0, "Connection timeout")
            try:
                self.bootloader.disconnect()
            except Exception:
                pass
            self.bootloader = None
            self._on_op_end()

    def _connect(self):
        """Connect to the CAN or UART device."""
        interface = self.interface_var.get()
        raw = self.channel_var.get().strip() if self.channel_var.get() else ""

        if not raw:
            channel = 0
        else:
            try:
                channel = int(raw)
            except ValueError:
                channel = raw

        try:
            bitrate = int(self.bitrate_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid bitrate value")
            return

        if interface == "virtual" and not self._simulator_running:
            self.log("Virtual bus requires --sim flag or an external simulator process", "error")
            return

        self.log(f"Connecting to {interface}:{channel} at {bitrate} bps...", "info")
        self._on_op_start()
        self.status_label.config(text=f"Connecting... (timeout {CONNECTION_TIMEOUT}s)")

        self._connect_timer = self.root.after(
            int(CONNECTION_TIMEOUT * 1000), self._connect_timeout
        )

        def do_connect():
            local_bl = None
            try:
                # ── UART pre-handshake (if enabled) ──────────────────────────
                if interface == "uart" and self.pre_enable_var.get():
                    try:
                        pre_baud = int(self.pre_uart_baud_var.get())
                    except ValueError:
                        self.root.after(0, lambda: self.log(
                            "UART pre-handshake: invalid pre-handshake baud", "error"))
                        self.root.after(0, self._cancel_connect_timer)
                        self.root.after(0, self._on_op_end)
                        return

                    # Create UARTBootloader with pre-handshake baud + parity for init
                    local_bl = UARTBootloader(port=str(channel), baudrate=pre_baud,
                                              parity=self.uart_parity_var.get())
                    success = self._send_uart_prehandshake_and_rebaud(
                        local_bl, pre_baud, bitrate
                    )
                    if not success:
                        self.root.after(0, self._cancel_connect_timer)
                        self.root.after(0, lambda: self._update_progress(0, "Pre-handshake failed"))
                        if local_bl:
                            try:
                                local_bl.disconnect()
                            except Exception:
                                pass
                        self.root.after(0, self._on_op_end)
                        return

                    # local_bl is now at main_baud — reuse it
                    self.bootloader = local_bl

                    # Small delay after pre-handshake
                    import time
                    time.sleep(0.3)

                else:
                    # ── Create bootloader without pre-handshake ────────────────
                    if interface == "uart":
                        local_bl = UARTBootloader(port=str(channel), baudrate=bitrate,
                                                  parity=self.uart_parity_var.get())
                    else:
                        local_bl = CANBootloader(interface=interface, channel=channel, bitrate=bitrate)

                    local_bl.connect()
                    self.bootloader = local_bl

                # ── CAN pre-handshake (if enabled) ──────────────────────────
                if interface != "uart" and self.pre_enable_var.get():
                    self.root.after(0, lambda: self.log("Sending CAN pre-handshake message...", "info"))
                    success = self._send_can_prehandshake(local_bl)
                    if not success:
                        self.root.after(0, self._cancel_connect_timer)
                        self.root.after(0, lambda: self._update_progress(0, "Pre-handshake failed"))
                        if self.bootloader:
                            try:
                                self.bootloader.disconnect()
                            except Exception:
                                pass
                            self.bootloader = None
                        self.root.after(0, self._on_op_end)
                        return

                    import time
                    self.root.after(0, lambda: self.log("Waiting 500ms after pre-handshake...", "info"))
                    time.sleep(0.5)

                # ────────────────────────────────────────────────────────────

                # Cancel timeout timer
                def _cancel_timer():
                    if self._connect_timer is not None:
                        self.root.after_cancel(self._connect_timer)
                        self._connect_timer = None
                self.root.after(0, _cancel_timer)

                # Sync (retry once on failure)
                self.root.after(0, lambda: self.log("Syncing with bootloader...", "info"))
                try:
                    local_bl.sync()
                except (NackError, TimeoutError):
                    self.root.after(0, lambda: self.log("Sync failed, retrying once...", "warn"))
                    import time as _time
                    _time.sleep(0.3)
                    local_bl.sync()
                self.root.after(0, lambda: self.log("Sync OK", "ok"))

                # Get device info
                self.root.after(0, lambda: self.log("Getting device info...", "info"))
                version, commands = local_bl.get()
                self.root.after(0, lambda v=version:
                    self.info_vars["version"].set(f"{v >> 4}.{v & 0x0F}"))
                self.root.after(0, lambda cmds=commands:
                    self.info_vars["commands"].set(" ".join(f"0x{c:02X}" for c in cmds)))

                chip_id = local_bl.get_id()
                self.root.after(0, lambda cid=chip_id:
                    self.info_vars["chip_id"].set(f"0x{cid:04X}"))

                cinfo = lookup_chip(chip_id)
                self.root.after(0, lambda ci=cinfo:
                    self.info_vars["chip_name"].set(
                        ci.name if ci.pages > 0 else f"{ci.name} (?)"))
                if cinfo.pages > 0:
                    total_flash = cinfo.page_size * cinfo.pages
                    self.root.after(0, lambda ci=cinfo:
                        self.info_vars["flash_page"].set(f"{_format_size(ci.page_size)}"))
                    self.root.after(0, lambda sz=total_flash:
                        self.info_vars["flash_size"].set(_format_size(sz)))
                    # Only apply the default Start Addr (FLASH_BASE + page_size)
                    # when no HEX is loaded; otherwise keep the firmware start
                    # address that was taken from the HEX file.
                    if self.hex_path is None:
                        FLASH_BASE = 0x08000000
                        default_start = FLASH_BASE + cinfo.page_size
                        self.root.after(0, lambda a=default_start:
                            self.start_addr_var.set(f"0x{a:08X}"))

                self.connected = True
                self.root.after(0, self._set_controls_state, True)
                self.root.after(0, lambda: self.log("Connected successfully!", "ok"))
                self.root.after(0, self._start_ping_monitor)

            except BootloaderError as e:
                err_msg = str(e)
                self.root.after(0, lambda msg=err_msg:
                    self.log(f"Connection failed: {msg}", "error"))
                self.root.after(0, lambda: self._update_progress(0, "Connection failed"))
                self.root.after(0, self._cancel_connect_timer)
                if self.bootloader:
                    try:
                        self.bootloader.disconnect()
                    except Exception:
                        pass
                    self.bootloader = None
            except Exception as e:
                err_msg = str(e)
                self.root.after(0, lambda msg=err_msg:
                    self.log(f"Connection error: {msg}", "error"))
                self.root.after(0, lambda: self._update_progress(0, "Connection error"))
                self.root.after(0, self._cancel_connect_timer)
                if self.bootloader:
                    try:
                        self.bootloader.disconnect()
                    except Exception:
                        pass
                    self.bootloader = None
                elif local_bl is not None:
                    try:
                        if hasattr(local_bl, 'bus') and local_bl.bus is not None:
                            local_bl.bus.shutdown()
                        elif hasattr(local_bl, 'ser') and local_bl.ser is not None:
                            local_bl.ser.close()
                    except Exception:
                        pass
            finally:
                self.root.after(0, self._on_op_end)

        threading.Thread(target=do_connect, daemon=True).start()

    def _cancel_connect_timer(self):
        """Cancel the connection timeout timer (must be called from main thread)."""
        if self._connect_timer is not None:
            self.root.after_cancel(self._connect_timer)
            self._connect_timer = None

    def _disconnect(self):
        """Disconnect from the device."""
        self._stop_ping_monitor()

        if self._connect_timer is not None:
            self.root.after_cancel(self._connect_timer)
            self._connect_timer = None

        self.log("Disconnecting...", "info")
        if self.bootloader:
            try:
                self.bootloader.disconnect()
            except Exception as e:
                self.log(f"Disconnect error: {e}", "error")
            finally:
                self.bootloader = None

        self.connected = False
        self._set_controls_state(connected=False)

        for key in self.info_vars:
            self.info_vars[key].set("--")

        self.progress_var.set(0)
        self.status_label.config(text="Disconnected")
        self.log("Disconnected", "warn")

    def _scan_devices(self):
        """Scan for available channels/ports for the currently selected interface.

        Re-detects interface types first, then refreshes the channel list.
        """
        interface = self.interface_var.get()
        self.log(f"Scanning for {interface} devices...", "info")
        self.scan_btn.config(state="disabled", text="Scanning...")
        self.root.update_idletasks()

        def do_scan():
            # Step 1: Re-detect available interfaces so Scan picks up newly
            #         plugged devices without requiring a GUI restart.
            available = []
            self._has_candle_devices = False
            self._has_com_ports = False

            try:
                configs = can.detect_available_configs('candle')
                if configs:
                    available.append("candle")
                    self._has_candle_devices = True
            except Exception:
                pass

            try:
                import serial.tools.list_ports
                ports = list(serial.tools.list_ports.comports())
                if ports:
                    available.append("slcan")
                    available.append("uart")
                    self._has_com_ports = True
            except Exception:
                pass

            if available:
                self.root.after(0, lambda a=available: self.interface_combo.config(values=a))

            # Step 2: Scan channels for the currently selected interface
            found = []

            if interface == "virtual":
                found = ["test_bus"]

            elif interface in ("slcan", "uart"):
                try:
                    import serial.tools.list_ports
                    ports = list(serial.tools.list_ports.comports())
                    found = [p.device for p in ports]
                    if found:
                        self.root.after(0, lambda f=found: self.log(
                            f"  Found COM ports: {', '.join(f)}", "ok"))
                    else:
                        self.root.after(0, lambda: self.log(
                            "  No COM ports detected. Check that your adapter is connected "
                            "and drivers are installed.", "warn"))
                except ImportError:
                    self.root.after(0, lambda: self.log(
                        "  'pyserial' module not found. Install: pip install pyserial", "error"))
                except Exception as e:
                    self.root.after(0, lambda msg=str(e): self.log(
                        f"  Error enumerating COM ports: {msg}", "error"))

            elif interface == "candle":
                try:
                    configs = can.detect_available_configs('candle')
                    found = [str(cfg.get('channel', '')) for cfg in configs]
                    if found:
                        self.root.after(0, lambda f=found: self.log(
                            f"  Found candle device channels: {', '.join(f)}", "ok"))
                    else:
                        self.root.after(0, lambda: self.log(
                            "  No candleLight devices detected. "
                            "Check USB connection and drivers.", "warn"))
                except Exception as e:
                    self.root.after(0, lambda msg=str(e): self.log(
                        f"  Error scanning candle devices: {msg}", "error"))

            self.root.after(0, lambda f=found: self._scan_complete(f))

        threading.Thread(target=do_scan, daemon=True).start()

    def _scan_complete(self, channels):
        """Called when scan finishes."""
        self.channel_combo.config(values=channels)
        if channels:
            self.channel_var.set(channels[0])
            self.log("  Selected: " + channels[0], "ok")
        else:
            self.channel_var.set("")
            self.log("  No devices found", "warn")
        self.scan_btn.config(state="normal", text="Scan")

    def _browse_hex(self):
        """Open file dialog to select a HEX file."""
        path = filedialog.askopenfilename(
            title="Select firmware HEX file",
            filetypes=[("Intel HEX files", "*.hex"), ("All files", "*.*")]
        )
        if path:
            self.hex_path = path
            self.hex_path_var.set(path)
            self.log(f"Selected: {path}", "info")

            try:
                from intelhex import IntelHex
                ih = IntelHex(path)
                segments = ih.segments()
                total = sum(end - start for start, end in segments)
                self.hex_data = ih
                self.hex_segments = segments
                self.hex_total_size = total

                # Auto-fill Start Addr from the firmware start address in the HEX
                if segments:
                    start = segments[0][0]
                    self.start_addr_var.set(f"0x{start:08X}")
                    self.log(f"  Start Addr set to 0x{start:08X} (from HEX)", "info")

                self.log(f"  Segments: {segments}", "info")
                self.log(f"  Total size: {_format_size(total)}", "info")
            except Exception as e:
                self.log(f"  Error reading hex: {e}", "error")
                self.hex_data = None

    def _erase_flash(self):
        """Erase flash memory."""
        if not self._check_connected():
            return
        if not messagebox.askyesno("Confirm Erase", "Erase all flash memory?"):
            return
        self.log("Erasing flash memory...", "warn")
        self._run_operation(self._do_erase)

    def _do_erase(self):
        self._mark_activity()
        try:
            self.root.after(0, lambda: self._update_progress(10, "Erasing..."))
            self.bootloader.erase_memory(global_erase=True)
            self.root.after(0, lambda: self._update_progress(100, "Erase complete"))
            self.root.after(0, lambda: self.log("Flash erased successfully", "ok"))
        except BootloaderError as e:
            err = str(e)
            self.root.after(0, lambda msg=err: self.log(f"Erase failed: {msg}", "error"))
            self.root.after(0, lambda: self._update_progress(0, "Erase failed"))

    def _program_flash(self):
        """Program flash from hex file."""
        if not self._check_connected():
            return
        if not self._check_hex():
            return

        if not messagebox.askyesno("Confirm Program",
                                    f"Program {_format_size(self.hex_total_size)} from\n"
                                    f"{os.path.basename(self.hex_path)}?"):
            return

        self.log(f"Programming {self.hex_path}...", "info")
        self._run_operation(self._do_program)

    def _do_program(self):
        def progress(p, s):
            self.root.after(0, lambda: self._update_progress(p, s))
            self.root.after(0, lambda: self.log(f"  {s}", "info"))

        try:
            if self._prog_erase_var.get():
                self.root.after(0, lambda: self.log("Erasing flash before programming...", "warn"))
                self.root.after(0, lambda: self._update_progress(0, "Erasing..."))
                self.bootloader.erase_memory(global_erase=True)
                self.root.after(0, lambda: self.log("Erase OK", "ok"))

            self.bootloader.program_hex(self.hex_path, progress_callback=progress)
            self.root.after(0, lambda: self.log("Programming complete!", "ok"))

            if self._prog_verify_var.get():
                self.root.after(0, lambda: self.log("Verifying after programming...", "info"))
                self._do_verify_inner()

            if self._prog_start_var.get():
                try:
                    addr = int(self.start_addr_var.get(), 16)
                except ValueError:
                    self.root.after(0, lambda: self.log(
                        "Start after program: invalid Start Addr — skipping Go", "warn"))
                else:
                    self.root.after(0, lambda a=addr: self.log(
                        f"Sending Go to 0x{a:08X}...", "warn"))
                    self.bootloader.go(addr)
                    self.root.after(0, lambda: self.log("Go command sent — device should restart", "ok"))

        except BootloaderError as e:
            err = str(e)
            self.root.after(0, lambda msg=err: self.log(f"Programming failed: {msg}", "error"))
            self.root.after(0, lambda: self._update_progress(0, "Programming failed"))
        except Exception as e:
            err = str(e)
            self.root.after(0, lambda msg=err: self.log(f"Unexpected error: {msg}", "error"))
            self.root.after(0, lambda: self._update_progress(0, "Error"))

    def _do_verify_inner(self):
        """Verify flash against hex file (called from _do_program background thread)."""
        from intelhex import IntelHex
        ih = IntelHex(self.hex_path)
        segments = ih.segments()
        total = sum(end - start for start, end in segments)
        verified = 0
        errors = 0

        for start_addr, end_addr in segments:
            addr = start_addr
            while addr < end_addr:
                chunk_size = min(256, end_addr - addr)
                expected = ih.tobinstr(addr, addr + chunk_size - 1)
                if isinstance(expected, str):
                    expected = expected.encode('latin-1')

                actual = self.bootloader.read_memory(addr, chunk_size)

                if expected != actual:
                    for i in range(chunk_size):
                        if i < len(expected) and i < len(actual):
                            if expected[i] != actual[i]:
                                errors += 1
                                if errors <= 10:
                                    e_val = f"{expected[i]:02X}"
                                    a_val = f"{actual[i]:02X}"
                                    cur_addr = addr + i
                                    self.root.after(0, lambda a=cur_addr, ex=e_val, ax=a_val:
                                        self.log(
                                            f"  Verify error at 0x{a:08X}: "
                                            f"expected 0x{ex}, got 0x{ax}", "error"))

                verified += chunk_size
                pct = int(verified * 100 / total) if total > 0 else 0
                cur_addr = addr
                self.root.after(0, lambda p=pct, a=cur_addr:
                    self._update_progress(p, f"Verifying 0x{a:08X}..."))
                addr += chunk_size

        if errors == 0:
            self.root.after(0, lambda: self.log("Verify OK — flash matches hex file!", "ok"))
            self.root.after(0, lambda: self._update_progress(100, "Verify OK"))
        else:
            self.root.after(0, lambda cnt=errors: self.log(f"Verify FAILED: {cnt} errors", "error"))
            self.root.after(0, lambda: self._update_progress(100, "Verify FAILED"))

    def _read_memory(self):
        """Read memory and save to hex file."""
        if not self._check_connected():
            return

        try:
            addr = int(self.start_addr_var.get(), 16)
        except ValueError:
            messagebox.showerror("Error", "Invalid start address")
            return

        total_flash = self._get_flash_size()
        if total_flash > 0:
            FLASH_BASE = 0x08000000
            if FLASH_BASE <= addr < FLASH_BASE + total_flash:
                offset = addr - FLASH_BASE
                size = total_flash - offset
            else:
                size = total_flash
            if size <= 0:
                size = total_flash
            self.log(f"Using flash size from chip database: {_format_size(total_flash)}", "info")
        else:
            size_str = tk.simpledialog.askstring(
                "Read Size",
                f"Enter number of bytes to read (starting at 0x{addr:08X}):",
                initialvalue="0x1000"
            )
            if not size_str:
                return
            try:
                if size_str.startswith("0x") or size_str.startswith("0X"):
                    size = int(size_str, 16)
                else:
                    size = int(size_str)
            except ValueError:
                messagebox.showerror("Error", "Invalid size value")
                return

        if size <= 0:
            messagebox.showerror("Error", "Size must be positive")
            return

        if size > 1024 * 1024:
            if not messagebox.askyesno("Large Read",
                                        f"Read {_format_size(size)}? This may take a while."):
                return

        path = filedialog.asksaveasfilename(
            title="Save memory dump as",
            defaultextension=".hex",
            filetypes=[("Intel HEX files", "*.hex"), ("All files", "*.*")]
        )
        if not path:
            return

        self.log(f"Reading memory: 0x{addr:08X} size {_format_size(size)}", "info")
        self._run_operation(lambda: self._do_read(addr, size, path))

    def _get_flash_size(self) -> int:
        chip_id_str = self.info_vars["chip_id"].get()
        if chip_id_str.startswith("0x"):
            try:
                chip_id = int(chip_id_str, 16)
                ci = lookup_chip(chip_id)
                return ci.page_size * ci.pages
            except ValueError:
                pass
        return 0

    def _do_read(self, addr: int, size: int, path: str):
        try:
            def progress(p, s):
                self.root.after(0, lambda: self._update_progress(p, s))

            self.bootloader.read_to_hex(addr, size, path, progress_callback=progress)
            self.root.after(0, lambda p=path: self.log(f"Memory saved to {p}", "ok"))
        except BootloaderError as e:
            err = str(e)
            self.root.after(0, lambda msg=err: self.log(f"Read failed: {msg}", "error"))
            self.root.after(0, lambda: self._update_progress(0, "Read failed"))

    def _verify_flash(self):
        if not self._check_connected():
            return
        if not self._check_hex():
            return

        self.log("Verifying flash memory...", "info")
        self._run_operation(self._do_verify)

    def _do_verify(self):
        try:
            from intelhex import IntelHex
            ih = IntelHex(self.hex_path)
            segments = ih.segments()
            total = sum(end - start for start, end in segments)
            verified = 0
            errors = 0

            for start_addr, end_addr in segments:
                addr = start_addr
                while addr < end_addr:
                    chunk_size = min(256, end_addr - addr)
                    expected = ih.tobinstr(addr, addr + chunk_size - 1)
                    if isinstance(expected, str):
                        expected = expected.encode('latin-1')

                    actual = self.bootloader.read_memory(addr, chunk_size)

                    if expected != actual:
                        for i in range(chunk_size):
                            if i < len(expected) and i < len(actual):
                                if expected[i] != actual[i]:
                                    errors += 1
                                    if errors <= 10:
                                        e_val = f"{expected[i]:02X}"
                                        a_val = f"{actual[i]:02X}"
                                        cur_addr = addr + i
                                        self.root.after(0, lambda a=cur_addr, ex=e_val, ax=a_val:
                                            self.log(
                                                f"  Verify error at 0x{a:08X}: "
                                                f"expected 0x{ex}, got 0x{ax}", "error"))

                    verified += chunk_size
                    pct = int(verified * 100 / total)
                    cur_addr = addr
                    self.root.after(0, lambda p=pct, a=cur_addr:
                        self._update_progress(p, f"Verifying 0x{a:08X}..."))
                    addr += chunk_size

            if errors == 0:
                self.root.after(0, lambda: self.log("Verify OK — flash matches hex file!", "ok"))
                self.root.after(0, lambda: self._update_progress(100, "Verify OK"))
            else:
                self.root.after(0, lambda cnt=errors: self.log(f"Verify FAILED: {cnt} errors", "error"))
                self.root.after(0, lambda cnt=errors: self._update_progress(100, f"Verify FAILED: {cnt} errors"))

        except BootloaderError as e:
            err = str(e)
            self.root.after(0, lambda msg=err: self.log(f"Verify failed: {msg}", "error"))
            self.root.after(0, lambda: self._update_progress(0, "Verify failed"))

    def _go_cmd(self):
        if not self._check_connected():
            return

        try:
            addr = int(self.start_addr_var.get(), 16)
        except ValueError:
            messagebox.showerror("Error", "Invalid start address")
            return

        if not messagebox.askyesno("Confirm Go",
                                    f"Start application at 0x{addr:08X}?"):
            return

        self.log(f"Sending Go command to 0x{addr:08X}...", "warn")
        self._run_operation(lambda: self._do_go(addr))

    def _do_go(self, addr: int):
        try:
            self.bootloader.go(addr)
            self.root.after(0, lambda: self.log("Go command sent — device should restart", "ok"))
            self.root.after(0, lambda: self._update_progress(100, "Go OK"))
        except BootloaderError as e:
            err = str(e)
            self.root.after(0, lambda msg=err: self.log(f"Go failed: {msg}", "error"))
            self.root.after(0, lambda: self._update_progress(0, "Go failed"))

    # ─── Utilities ─────────────────────────────────────────────────────────────

    def _check_connected(self) -> bool:
        if not self.connected or self.bootloader is None:
            messagebox.showwarning("Not Connected", "Connect to device first")
            return False
        return True

    def _check_hex(self) -> bool:
        if not self.hex_path:
            messagebox.showwarning("No HEX File", "Select a firmware HEX file first")
            return False
        return True

    def _run_operation(self, target):
        self._on_op_start()
        threading.Thread(target=lambda: self._run_wrapper(target), daemon=True).start()

    def _run_wrapper(self, target):
        try:
            target()
        except Exception as e:
            err = str(e)
            self.root.after(0, lambda msg=err: self.log(f"Error: {msg}", "error"))
            self.root.after(0, lambda: self._update_progress(0, "Error"))
        finally:
            self.root.after(500, self._on_op_end)

    def _on_close(self):
        if self._connect_timer is not None:
            self.root.after_cancel(self._connect_timer)
            self._connect_timer = None
        if self.connected:
            self._disconnect()
        self.root.destroy()

    def run(self):
        """Start the GUI main loop."""
        self.root.mainloop()


# ─── Entry point ───────────────────────────────────────────────────────────────

def main():
    import argparse

    # Enable Windows DPI awareness so Tk renders crisp (non-blurry) text on
    # high-DPI / scaled displays instead of being bitmap-stretched by the OS.
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            try:
                ctypes.windll.user32.SetProcessDPIAware()
            except (AttributeError, OSError):
                pass

    parser = argparse.ArgumentParser(description="CAN Flash Tool GUI")
    parser.add_argument("--sim", action="store_true",
                        help="Start bootloader simulator in background (for testing without hardware)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.WARNING)
    app = CANFlashToolGUI()

    if args.sim:
        if not _SIM_AVAILABLE:
            print("ERROR: --sim requires can_bootloader_sim.py (not found)")
            exit(1)
        print("Starting bootloader simulator...")
        sim_channel = "test_bus"
        sim_bitrate = 125000
        app._simulator_running = True
        app.channel_combo.config(values=[sim_channel])
        app.channel_var.set(sim_channel)
        app.interface_var.set("virtual")

        def run_sim():
            try:
                from can_bootloader_sim import CANBootloaderSimulator
                sim = CANBootloaderSimulator(channel=sim_channel, bitrate=sim_bitrate)
                app._simulator = sim
                sim.run()
            except Exception as e:
                print(f"Simulator error: {e}")
            finally:
                app._simulator_running = False

        import threading
        t = threading.Thread(target=run_sim, daemon=True)
        app._simulator_thread = t
        t.start()
        import time
        time.sleep(0.5)

        print(f"Simulator started on virtual bus '{sim_channel}'")
        print("Press 'Connect' in the GUI to connect to the simulator")

    app.run()


if __name__ == '__main__':
    main()
