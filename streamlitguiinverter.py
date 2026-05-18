"""
╔══════════════════════════════════════════════════════════╗
║  SolaX X1 Hybrid G4 — Modbus RTU Dashboard              ║
║  Protocol V3.21  |  Single-Phase X1                     ║
║  Modes: Live Serial  /  File-Only (no hardware needed)  ║
╚══════════════════════════════════════════════════════════╝

Tabs:
  1. 📊  Live Dashboard      — real-time poll → metrics + charts
  2. 🔍  Data Interpreter    — full register decode table
  3. 🛠   Manual Frame Test   — custom frame builder + sender
  4. 🧩  Hex Decoder          — paste raw hex / upload Excel

Run:
  pip install streamlit pyserial crcmod pandas openpyxl
  streamlit run solax_x1_gui.py
"""

# ── std-lib ──────────────────────────────────────────────────
import time
import struct
from datetime import datetime
from collections import deque

# ── third-party ──────────────────────────────────────────────
import streamlit as st
import crcmod
import pandas as pd

# pyserial is optional — app works without hardware
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# ═════════════════════════════════════════════════════════════
# PAGE CONFIG
# ═════════════════════════════════════════════════════════════
st.set_page_config(
    page_title="SolaX X1 Monitor"
)

# ── Custom CSS ───────────────────────────────────────────────
st.markdown("""
<style>
  /* Tighten metric cards */
  [data-testid="metric-container"] {
      background: #1a1f2e;
      border: 1px solid #2d3448;
      border-radius: 10px;
      padding: 12px 16px;
  }
  [data-testid="stMetricLabel"]  { font-size: 0.72rem; color: #8892a4; }
  [data-testid="stMetricValue"]  { font-size: 1.35rem; font-weight: 700; }
  [data-testid="stMetricDelta"]  { font-size: 0.72rem; }

  /* Tab bar */
  [data-testid="stTabs"] button[role="tab"] { font-weight: 600; }

  /* Sidebar */
  [data-testid="stSidebar"] { background: #111827; }

  /* Section headers */
  .section-header {
      font-size: 0.78rem;
      font-weight: 700;
      letter-spacing: 0.12em;
      text-transform: uppercase;
      color: #64748b;
      margin: 18px 0 8px 0;
      padding-bottom: 4px;
      border-bottom: 1px solid #2d3448;
  }

  /* Status badge */
  .badge-ok   { background:#16a34a22; color:#4ade80;
                border:1px solid #16a34a; border-radius:20px;
                padding:2px 10px; font-size:0.78rem; font-weight:600; }
  .badge-warn { background:#d9770622; color:#fb923c;
                border:1px solid #d97706; border-radius:20px;
                padding:2px 10px; font-size:0.78rem; font-weight:600; }
  .badge-err  { background:#dc262622; color:#f87171;
                border:1px solid #dc2626; border-radius:20px;
                padding:2px 10px; font-size:0.78rem; font-weight:600; }

  /* SOC bar */
  .soc-wrap { background:#1e2535; border-radius:8px;
              height:22px; margin:4px 0 14px 0; overflow:hidden; }
  .soc-fill { height:100%; border-radius:8px; display:flex;
              align-items:center; justify-content:center;
              color:#fff; font-size:12px; font-weight:700;
              transition: width 0.6s ease; }
</style>
""", unsafe_allow_html=True)

# ═════════════════════════════════════════════════════════════
# CRC
# ═════════════════════════════════════════════════════════════
_crc16 = crcmod.predefined.mkCrcFun("modbus")

# ═════════════════════════════════════════════════════════════
# X1-SPECIFIC REGISTER MAPS  (protocol V3.21)
# ═════════════════════════════════════════════════════════════

# ── 0x04 Read Input Registers ────────────────────────────────
# (addr, short_name, description, scale, unit, dtype, category)
INPUT_REGS = [
    # Grid / AC  (X1 single-phase)
    (0x0000, "Grid Voltage",      "Grid RMS Voltage",               0.1,   "V",   "uint16", "Grid"),
    (0x0001, "Grid Current",      "Grid RMS Current",               0.1,   "A",   "int16",  "Grid"),
    (0x0002, "Grid Power",        "Grid Active Power",              1.0,   "W",   "int16",  "Grid"),
    (0x0007, "Grid Frequency",    "Grid Frequency",                 0.01,  "Hz",  "uint16", "Grid"),
    (0x001A, "Grid Status",       "On-Grid / Off-Grid",             1.0,   "",    "uint16", "Grid"),

    # PV strings
    (0x0003, "PV1 Voltage",       "PV String 1 Voltage",            0.1,   "V",   "uint16", "PV"),
    (0x0004, "PV2 Voltage",       "PV String 2 Voltage",            0.1,   "V",   "uint16", "PV"),
    (0x0005, "PV1 Current",       "PV String 1 Current",            0.1,   "A",   "uint16", "PV"),
    (0x0006, "PV2 Current",       "PV String 2 Current",            0.1,   "A",   "uint16", "PV"),
    (0x000A, "PV1 Power",         "DC Power String 1",              1.0,   "W",   "uint16", "PV"),
    (0x000B, "PV2 Power",         "DC Power String 2",              1.0,   "W",   "uint16", "PV"),

    # Inverter
    (0x0008, "Inv Temperature",   "Radiator Temperature",           1.0,   "°C",  "int16",  "Inverter"),
    (0x0009, "Run Mode",          "Inverter Run Mode",              1.0,   "",    "uint16", "Inverter"),
    (0x001B, "MPPT Count",        "Number of MPPT Channels",        1.0,   "",    "uint16", "Inverter"),

    # Battery
    (0x0014, "Bat Voltage",       "Battery Voltage",                0.1,   "V",   "int16",  "Battery"),
    (0x0015, "Bat Current",       "Battery Current (+chg/−dis)",    0.1,   "A",   "int16",  "Battery"),
    (0x0016, "Bat Power",         "Battery Power (+chg/−dis)",      1.0,   "W",   "int16",  "Battery"),
    (0x0018, "Bat Temperature",   "Battery Temperature",            1.0,   "°C",  "int16",  "Battery"),
    (0x001C, "SOC",               "State of Charge",                1.0,   "%",   "uint16", "Battery"),
    (0x0019, "BDC Status",        "Battery Direction",              1.0,   "",    "uint16", "Battery"),
    (0x0017, "BMS Connected",     "BMS Connection State",           1.0,   "",    "uint16", "Battery"),
    (0x0024, "BMS Max Chg I",     "BMS Charge Max Current",         0.1,   "A",   "uint16", "Battery"),
    (0x0025, "BMS Max Dis I",     "BMS Discharge Max Current",      0.1,   "A",   "uint16", "Battery"),
    (0x00BA, "Bat Temp High",     "Battery High Temperature",       0.1,   "°C",  "int16",  "Battery"),
    (0x00BB, "Bat Temp Low",      "Battery Low Temperature",        0.1,   "°C",  "int16",  "Battery"),
    (0x00BC, "Cell V High",       "Highest Cell Voltage",           0.001, "V",   "uint16", "Battery"),
    (0x00BD, "Cell V Low",        "Lowest Cell Voltage",            0.001, "V",   "uint16", "Battery"),
    (0x00BE, "BMS SOC",           "BMS User SOC",                   1.0,   "%",   "uint16", "Battery"),
    (0x00BF, "BMS SOH",           "BMS User SOH",                   1.0,   "%",   "uint16", "Battery"),

    # Off-grid (X1)
    (0x004C, "Off-grid Voltage",  "Off-grid Output Voltage (X1)",   0.1,   "V",   "uint16", "Off-grid"),
    (0x004D, "Off-grid Current",  "Off-grid Output Current (X1)",   0.1,   "A",   "uint16", "Off-grid"),
    (0x004E, "Off-grid Power",    "Off-grid Output Power (X1)",     1.0,   "VA",  "uint16", "Off-grid"),
    (0x004F, "Off-grid Freq",     "Off-grid Output Frequency (X1)", 0.01,  "Hz",  "uint16", "Off-grid"),

    # Energy counters
    (0x0050, "Yield Today",       "Today Energy — AC Port",         0.1,   "kWh", "uint16", "Energy"),
    (0x0096, "Solar Today",       "Solar Energy Today",             0.1,   "kWh", "uint16", "Energy"),
    (0x0020, "Bat Chg Today",     "Battery Output Energy Today",    0.1,   "kWh", "uint16", "Energy"),
    (0x0023, "Bat Dis Today",     "Battery Input Energy Today",     0.1,   "kWh", "uint16", "Energy"),
    (0x0091, "ECharge Today",     "Charge Energy Today (AC)",       0.1,   "kWh", "uint16", "Energy"),
    (0x0090, "Off-grid Today",    "Off-grid Yield Today",           0.1,   "kWh", "uint16", "Energy"),

    # Feed-in  (32-bit: 0x0046=LSB, 0x0047=MSB)
    (0x0046, "Feed-in Power",     "Feed-in Power (Meter/CT)",       1.0,   "W",   "int32",  "Grid"),

    # Faults
    (0x003E, "PCS Fault",         "PCS Major Fault Code",           1.0,   "",    "uint16", "Faults"),
    (0x003F, "Bat Fault",         "Battery Major Fault Code",       1.0,   "",    "uint16", "Faults"),
    (0x0040, "Inv Fault LSB",     "Inverter Error Code — LSB",      1.0,   "",    "uint16", "Faults"),
    (0x0041, "Inv Fault MSB",     "Inverter Error Code — MSB",      1.0,   "",    "uint16", "Faults"),
    (0x0043, "Mgr Fault",         "Manager Fault Code",             1.0,   "",    "uint16", "Faults"),
    (0x0044, "BMS Fault LSB",     "BMS Warning Code — LSB",         1.0,   "",    "uint16", "Faults"),
    (0x0045, "BMS Fault MSB",     "BMS Warning Code — MSB",         1.0,   "",    "uint16", "Faults"),

    # Remote control
    (0x0100, "Modbus PWR Ctrl",   "Remote Power Control Mode",      1.0,   "",    "uint16", "Control"),
    (0x011B, "Target SOC",        "Target SOC (remote ctrl)",       1.0,   "%",   "uint16", "Control"),
    (0x011F, "Force Chg Flag",    "Battery Force Charge Flag",      1.0,   "",    "uint16", "Control"),
    (0x0120, "BMS Relay State",   "BMS Relay State",                1.0,   "",    "uint16", "Control"),

    # X1-specific
    (0x011C, "Shutdown",          "Shutdown State (X1)",            1.0,   "",    "uint16", "Inverter"),
    (0x011D, "MicroGrid",         "MicroGrid State (X1)",           1.0,   "",    "uint16", "Inverter"),
    (0x011B, "bPVConnMode",       "PV Connection Mode (X1)",        1.0,   "",    "uint16", "Inverter"),
]

# ── 0x03 Read Holding Registers ──────────────────────────────
HOLDING_REGS = [
    (0x0000, "Inverter SN[0]",    "Serial Number chars 1-2",        1.0,   "",    "ascii",  "Info"),
    (0x0007, "Factory[0]",        "Factory Name chars 1-2",         1.0,   "",    "ascii",  "Info"),
    (0x000E, "Module[0]",         "Module Name chars 1-2",          1.0,   "",    "ascii",  "Info"),
    (0x007D, "FW DSP Minor",      "DSP Firmware Minor Version",     1.0,   "",    "uint16", "Firmware"),
    (0x007E, "HW DSP",            "DSP Hardware Version",           1.0,   "",    "uint16", "Firmware"),
    (0x007F, "FW DSP Major",      "DSP Firmware Major Version",     1.0,   "",    "uint16", "Firmware"),
    (0x0080, "FW ARM Major",      "ARM Firmware Major Version",     1.0,   "",    "uint16", "Firmware"),
    (0x0082, "FW Modbus RTU",     "Modbus RTU Firmware Version",    1.0,   "",    "uint16", "Firmware"),
    (0x0083, "FW ARM Minor",      "ARM Firmware Minor Version",     1.0,   "",    "uint16", "Firmware"),
    (0x0085, "RTC Seconds",       "Real-Time Clock — Seconds",      1.0,   "s",   "uint16", "RTC"),
    (0x0086, "RTC Minutes",       "Real-Time Clock — Minutes",      1.0,   "min", "uint16", "RTC"),
    (0x0087, "RTC Hours",         "Real-Time Clock — Hours",        1.0,   "h",   "uint16", "RTC"),
    (0x0088, "RTC Days",          "Real-Time Clock — Days",         1.0,   "",    "uint16", "RTC"),
    (0x0089, "RTC Months",        "Real-Time Clock — Months",       1.0,   "",    "uint16", "RTC"),
    (0x008A, "RTC Years",         "Real-Time Clock — Years",        1.0,   "",    "uint16", "RTC"),
    (0x008B, "Use Mode",          "Solar Charger Use Mode",         1.0,   "",    "uint16", "Settings"),
    (0x008C, "Manual Mode",       "Manual Mode Setting",            1.0,   "",    "uint16", "Settings"),
    (0x008D, "Battery Type",      "Battery Type",                   1.0,   "",    "uint16", "Settings"),
    (0x008E, "Float Voltage",     "Lead-acid Float Voltage",        0.1,   "V",   "uint16", "Settings"),
    (0x008F, "Discharge CutV",    "Lead-acid Discharge Cut-off V",  0.1,   "V",   "uint16", "Settings"),
    (0x0090, "Max Chg I",         "Lead-acid Max Charge Current",   0.1,   "A",   "uint16", "Settings"),
    (0x0091, "Max Dis I",         "Lead-acid Max Discharge I",      0.1,   "A",   "uint16", "Settings"),
    (0x001D, "Safety Code",       "Safety Type Code",               1.0,   "",    "uint16", "Settings"),
    (0x001E, "MateBox Enable",    "MateBox Enable",                 1.0,   "",    "uint16", "Settings"),
    (0x00AF, "ModBus Address",    "ModBus RTU Slave Address",       1.0,   "",    "uint16", "Settings"),
    (0x00B0, "ModBus Baud",       "ModBus RTU Baud Rate Code",      1.0,   "",    "uint16", "Settings"),
    (0x00BA, "Inv Power Type",    "Inverter Power Type",            1.0,   "W",   "uint16", "Info"),
    (0x00BB, "Language",          "Display Language",               1.0,   "",    "uint16", "Settings"),
    (0x00BC, "EnableMPPT",        "MPPT Enable",                    1.0,   "",    "uint16", "Settings"),
    (0x0104, "wShadowFix",        "Shadow Fix Function Enable",     1.0,   "",    "uint16", "Settings"),
    (0x0105, "MachineType",       "Machine Type (1=X1, 3=X3)",      1.0,   "",    "uint16", "Info"),
    (0x0108, "Meter Function",    "Meter Function Enable",          1.0,   "",    "uint16", "Settings"),
    (0x0109, "Meter1 ID",         "Meter 1 Modbus ID",              1.0,   "",    "uint16", "Settings"),
    (0x010B, "Dir MeterCT1",      "Direction Meter/CT 1",           1.0,   "",    "uint16", "Settings"),
    (0x011C, "Shutdown",          "Shutdown Enable (X1)",           1.0,   "",    "uint16", "Control"),
    (0x011D, "MicroGrid",         "MicroGrid Enable (X1)",          1.0,   "",    "uint16", "Control"),
    (0x011E, "Self-use BackupEn", "Self-use Backup Enable",         1.0,   "",    "uint16", "Control"),
    (0x011F, "Backup SOC",        "Self-use Backup SOC",            1.0,   "%",   "uint16", "Control"),
    (0x0130, "Parallel Setting",  "Parallel Setting",               1.0,   "",    "uint16", "Control"),
    (0x0131, "ExtGen Enable",     "External Generator Enable",      1.0,   "",    "uint16", "Control"),
]

# ═════════════════════════════════════════════════════════════
# LOOKUP TABLES  (X1-specific where noted)
# ═════════════════════════════════════════════════════════════
RUN_MODE = {
    0: "⏳ Waiting",      1: "🔍 Checking",  2: "✅ Normal",
    3: "⚠️ Fault",        4: "🔴 Perm Fault", 5: "🔄 Update",
    6: "🏝 Off-grid Wait", 7: "🏝 Off-grid",  8: "🧪 Self-test",
    9: "💤 Idle",         10: "🕹 Standby",
}
BDC_STATUS  = {0: "🔋 Discharging", 1: "⚡ Charging", 2: "⏸ Stopped"}
GRID_STATUS = {0: "🟢 On-Grid",     1: "🔴 Off-Grid"}
BMS_CONN    = {0: "❌ Disconnected", 1: "✅ Connected"}
USE_MODE    = {0: "Self Use", 1: "Feed-in Priority", 2: "Back Up", 3: "Manual"}
MANUAL_MODE = {0: "Stop", 1: "Force Charge", 2: "Force Discharge"}
BAT_TYPE    = {0: "Lead Acid", 1: "Lithium"}
MOD_CTRL    = {0: "Disabled", 1: "Power Control",
               2: "Energy Control", 3: "SOC Target Control"}
BAUD_CODE   = {0: 115200, 1: 57600, 2: 56000, 3: 38400, 4: 19200, 5: 14400, 6: 9600}
DIRECTION   = {0: "Positive", 1: "Negative"}
SHADOW_FIX  = {0: "Off", 1: "Low", 2: "Middle", 3: "High"}
LANGUAGE    = {0: "English", 1: "German", 2: "French", 3: "Polish",
               4: "Spanish", 5: "Portuguese", 6: "Italian"}
FORCE_CHG   = {0: "No Action", 1: "Force Charge"}
BMS_RELAY   = {0: "OFF", 1: "ON"}
PARALLEL    = {0: "Free", 1: "Master", 2: "Slave"}
YESNO       = {0: "Disabled", 1: "Enabled"}
MACHINE     = {1: "X1", 3: "X3"}

# Name → lookup dict
_LOOKUP = {
    "Run Mode": RUN_MODE,       "BDC Status": BDC_STATUS,
    "Grid Status": GRID_STATUS, "BMS Connected": BMS_CONN,
    "Use Mode": USE_MODE,       "Manual Mode": MANUAL_MODE,
    "Battery Type": BAT_TYPE,   "Modbus PWR Ctrl": MOD_CTRL,
    "Dir MeterCT1": DIRECTION,  "wShadowFix": SHADOW_FIX,
    "Language": LANGUAGE,       "Force Chg Flag": FORCE_CHG,
    "BMS Relay State": BMS_RELAY, "Parallel Setting": PARALLEL,
    "EnableMPPT": YESNO,        "MateBox Enable": YESNO,
    "Meter Function": YESNO,    "ExtGen Enable": YESNO,
    "Shutdown": YESNO,          "MicroGrid": YESNO,
    "Self-use BackupEn": YESNO, "MachineType": MACHINE,
    "ModBus Baud": BAUD_CODE,   "Shutdown": YESNO,
}

# X1 Inverter Fault Bits (0x0040 LSB + 0x0041 MSB)
X1_FAULT_BITS = {
    0:  "TZ Protect",      1:  "Grid Lost",       2:  "Grid Volt",
    3:  "Grid Freq",       4:  "PV Volt",          5:  "Bus Volt",
    6:  "Bat Volt",        7:  "AC10min Volt",     8:  "DCI OCP",
    9:  "Reserve9",       10:  "SW OCP",           11: "RC OCP",
    12: "Isolation",      13:  "Temp Over",        14: "BatConnDir",
    15: "Missed CT",      16:  "Off-grid OL",      17: "Overload",
    18: "PV ConnDir",     19:  "Bat Power Low",    20: "Low Temp",
    22: "Charger Relay",  23:  "BMS Lost",         24: "Inner Comm",
    25: "Fan Fault",      26:  "Earth Relay",      27: "INV EEPROM",
    28: "RCD Fault",      29:  "Off-grid Relay",   30: "Grid Relay",
    31: "Other Device",
}

# BMS Warning Bits (0x0044 LSB + 0x0045 MSB)
BMS_FAULT_BITS = {
    0:  "External Err",    1:  "Internal Err",   2:  "OverVoltage",
    3:  "LowVoltage",      4:  "ChargeOCP",       5:  "DischargeOCP",
    6:  "TempHigh",        7:  "TempLow",         8:  "CellImbalance",
    9:  "HW Protect",     10:  "Circuit Fault",  11:  "ISO Fault",
    12: "VolSen Fault",   13:  "TempSen Fault",  14:  "CurSen Fault",
    15: "Relay Fault",    16:  "Type Unmatch",   17:  "Ver Unmatch",
    18: "MFR Unmatch",    19:  "SW Unmatch",      20: "M&S Unmatch",
    21: "CR NoRespond",   22:  "SW Protect",     24:  "SelfcheckErr",
    25: "TempdiffErr",    26:  "BreakFault",      27: "Flash Fault",
    28: "Precharge Fault",29:  "AirSwitch Break",
}

# ═════════════════════════════════════════════════════════════
# MODBUS HELPERS
# ═════════════════════════════════════════════════════════════

def build_frame(slave: int, fc: int, reg: int, count_or_val: int) -> bytes:
    """Build a Modbus RTU request frame with CRC."""
    body = bytes([slave, fc, (reg >> 8) & 0xFF, reg & 0xFF,
                  (count_or_val >> 8) & 0xFF, count_or_val & 0xFF])
    crc = _crc16(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def crc_ok(frame: bytes, fc: int) -> bool:
    if len(frame) < 4:
        return False
    if frame[1] != fc:
        return False
    calc = _crc16(frame[:-2])
    got  = frame[-2] | (frame[-1] << 8)
    return calc == got


def parse_words(resp: bytes) -> list:
    """Extract register words from a normal Modbus response."""
    if len(resp) < 5:
        return []
    n = resp[2]           # byte count
    data = resp[3: 3 + n]
    return [(data[i] << 8) | data[i + 1] for i in range(0, len(data) - 1, 2)]


def i16(v: int) -> int:
    return v if v < 0x8000 else v - 0x10000


def i32(lo: int, hi: int) -> int:
    c = (hi << 16) | lo
    return c if c < 0x80000000 else c - 0x100000000


def decode_reg(word_map: dict, addr: int, dtype: str, scale: float):
    """Convert raw word(s) → Python number."""
    if addr not in word_map:
        return None
    v = word_map[addr]
    if dtype == "int16":
        v = i16(v)
    elif dtype == "int32":
        lo, hi = v, word_map.get(addr + 1, 0)
        v = i32(lo, hi)
    elif dtype == "uint8hi":
        v = (v >> 8) & 0xFF
    elif dtype == "uint8lo":
        v = v & 0xFF
    return round(v * scale, 3)


def friendly(name: str, val, unit: str, raw: int) -> str:
    if val is None:
        return "—"
    lk = _LOOKUP.get(name)
    if lk is not None and raw in lk:
        return lk[raw]
    return f"{val} {unit}".strip()


def active_faults(word32: int, bit_map: dict) -> list:
    return [bit_map[b] for b in sorted(bit_map) if (word32 >> b) & 1]


# ── Serial helpers ────────────────────────────────────────────

def _send(port, baud, slave, fc, reg, count, timeout=1.0):
    """Send one Modbus frame, return (frame_bytes, response_bytes | None)."""
    frame = build_frame(slave, fc, reg, count)
    try:
        with serial.Serial(port, baud, timeout=timeout) as s:
            time.sleep(0.05)
            s.write(frame)
            expected = 5 + count * 2
            resp = s.read(expected)
        return frame, resp if resp else None
    except Exception as e:
        return frame, None


def poll_input_bulk(port, baud, slave):
    """Poll all X1 input registers in protocol-compliant chunks."""
    result = {}
    # Chunks: (start_reg, count)  — kept ≤ 50 per request, gap > 100 ms
    chunks = [
        (0x0000, 0x0030),   # Grid, PV, Battery core, energy basics
        (0x003E, 0x000A),   # Faults, Feed-in power
        (0x004C, 0x0008),   # Off-grid
        (0x0050, 0x0002),   # Yield today
        (0x008E, 0x0012),   # Energy, solar
        (0x00B8, 0x0010),   # BMS temps, cell V, SOC/SOH
        (0x0100, 0x0022),   # Remote control
        (0x011A, 0x000A),   # X1-specific flags
    ]
    for start, cnt in chunks:
        _, resp = _send(port, baud, slave, 0x04, start, cnt)
        if resp and crc_ok(resp, 0x04):
            for i, w in enumerate(parse_words(resp)):
                result[start + i] = w
        time.sleep(0.12)   # > 100 ms inter-frame gap
    return result


def poll_holding_bulk(port, baud, slave):
    result = {}
    chunks = [
        (0x0000, 0x0020),
        (0x007D, 0x0040),
        (0x00AF, 0x0020),
        (0x0100, 0x0040),
        (0x011A, 0x0020),
    ]
    for start, cnt in chunks:
        _, resp = _send(port, baud, slave, 0x03, start, cnt)
        if resp and crc_ok(resp, 0x03):
            for i, w in enumerate(parse_words(resp)):
                result[start + i] = w
        time.sleep(0.12)
    return result


# ═════════════════════════════════════════════════════════════
# UI HELPERS
# ═════════════════════════════════════════════════════════════

def soc_bar(pct: int):
    pct = max(0, min(100, int(pct)))
    if pct > 50:
        color = "#22c55e"
    elif pct > 20:
        color = "#f59e0b"
    else:
        color = "#ef4444"
    st.markdown(
        f'<div class="soc-wrap"><div class="soc-fill" style="width:{pct}%;background:{color};">'
        f'{pct}%</div></div>',
        unsafe_allow_html=True,
    )


def section(label: str):
    st.markdown(f'<div class="section-header">{label}</div>', unsafe_allow_html=True)


def badge(text: str, level: str = "ok"):
    cls = {"ok": "badge-ok", "warn": "badge-warn", "err": "badge-err"}.get(level, "badge-ok")
    st.markdown(f'<span class="{cls}">{text}</span>', unsafe_allow_html=True)


def metric_row(cols, items):
    """items: list of (label, value_str, delta=None)"""
    for col, (lbl, val, *rest) in zip(cols, items):
        delta = rest[0] if rest else None
        col.metric(lbl, val, delta)


def fault_expander(title: str, word32: int, bit_map: dict):
    faults = active_faults(word32, bit_map)
    if faults:
        with st.expander(f"⚠️ {title}  —  {len(faults)} active fault(s)"):
            for f in faults:
                st.markdown(f"- 🔴 **{f}**")
    else:
        st.markdown(f'<span class="badge-ok">✅ {title}: No Faults</span>', unsafe_allow_html=True)


# ═════════════════════════════════════════════════════════════
# SIDEBAR
# ═════════════════════════════════════════════════════════════

with st.sidebar:
    st.markdown("## ⚡ SolaX X1 Monitor")
    st.caption("Hybrid G4 · Single-Phase · Modbus RTU V3.21")
    st.divider()

    # Mode selector
    mode = st.radio("Connection Mode",
                    ["🔌 Live Serial", "📂 File / Demo"],
                    help="'File / Demo' works without hardware — load an Excel export or use demo data.")
    live_mode = mode.startswith("🔌")

    st.divider()

    if live_mode:
        if not SERIAL_AVAILABLE:
            st.error("pyserial not installed.\n`pip install pyserial`")
            live_mode = False
        else:
            ports = [p.device for p in serial.tools.list_ports.comports()]
            if not ports:
                ports = ["COM4"]
            sel_port = st.selectbox("Serial Port", ports,
                                    index=ports.index("COM4") if "COM4" in ports else 0)
            sel_baud = st.selectbox("Baud Rate",
                                    [9600, 19200, 38400, 57600, 115200],
                                    index=[9600, 19200, 38400, 57600, 115200].index(9600),
                                    help="Protocol default: 19200. Your script uses 9600 — match your inverter setting.")
            sel_slave = st.number_input("Slave ID", 1, 247, 1)
            sel_timeout = st.slider("Response Timeout (s)", 0.5, 5.0, 1.0, 0.5)
    else:
        st.info("Load an Excel export from your script, or click **Load Demo Data** below.")
        sel_port, sel_baud, sel_slave, sel_timeout = "COM4", 9600, 1, 1.0

    st.divider()
    st.caption("⚠️ Min interval between frames: **1 s**\n"
               "⚠️ EEprom-★ registers: limited write cycles")

# ── Demo data (simulates a normal X1 running on solar + grid) ─

DEMO_DATA = {
    # Grid
    0x0000: 2300, 0x0001: 150,  0x0002: 345,   0x0007: 5001,
    0x001A: 0,
    # PV
    0x0003: 3850, 0x0004: 3720, 0x0005: 85,    0x0006: 60,
    0x000A: 3270, 0x000B: 2232,
    # Inverter
    0x0008: 42,   0x0009: 2,    0x001B: 2,
    # Battery
    0x0014: 520,  0x0015: 80,   0x0016: 416,   0x0018: 28,
    0x001C: 72,   0x0019: 1,    0x0017: 1,
    0x0024: 250,  0x0025: 250,
    0x00BA: 285,  0x00BB: 272,
    0x00BC: 3650, 0x00BD: 3620,
    0x00BE: 72,   0x00BF: 98,
    # Off-grid
    0x004C: 0,    0x004D: 0,    0x004E: 0,     0x004F: 0,
    # Feed-in (32-bit, LSB first)
    0x0046: 0xFEE0, 0x0047: 0xFFFF,   # -288 W (importing)
    # Energy
    0x0050: 142,  0x0096: 187,  0x0020: 55,    0x0023: 70,
    0x0091: 45,   0x0090: 0,
    # Faults — all clear
    0x003E: 0,    0x003F: 0,    0x0040: 0,     0x0041: 0,
    0x0043: 0,    0x0044: 0,    0x0045: 0,
    # Remote ctrl
    0x0100: 0,    0x011B: 0,    0x011F: 0,     0x0120: 1,
    # X1
    0x011C: 0,    0x011D: 0,
}

# ═════════════════════════════════════════════════════════════
# SESSION STATE  (history for mini-charts)
# ═════════════════════════════════════════════════════════════
if "history" not in st.session_state:
    st.session_state.history = {
        "ts": deque(maxlen=60),
        "pv_power": deque(maxlen=60),
        "bat_power": deque(maxlen=60),
        "grid_power": deque(maxlen=60),
        "soc": deque(maxlen=60),
    }

# ═════════════════════════════════════════════════════════════
# TABS
# ═════════════════════════════════════════════════════════════
tab1, tab2, tab3, tab4 = st.tabs([
    "📊  Live Dashboard",
    "🔍  Data Interpreter",
    "🛠   Manual Frame Test",
    "🧩  Hex / Excel Decoder",
])

# ╔═══════════════════════════════════════════════════════════╗
# ║  TAB 1 — LIVE DASHBOARD                                  ║
# ╚═══════════════════════════════════════════════════════════╝
with tab1:
    st.markdown("## 📊 Live Dashboard")

    ctrl_c1, ctrl_c2, ctrl_c3 = st.columns([2, 1, 1])
    with ctrl_c1:
        poll_interval = st.slider("Auto-refresh (s)", 3, 120, 10,
                                  key="poll_interval", label_visibility="collapsed")
        st.caption(f"Refresh every **{poll_interval} s**  |  min spec: 1 s")
    with ctrl_c2:
        auto = st.toggle("Auto-refresh", value=False)
    with ctrl_c3:
        manual_poll = st.button("🔄 Poll Now", use_container_width=True)

    if auto:
        time.sleep(poll_interval)
        st.rerun()

    # ── Acquire data ─────────────────────────────────────────
    regs = None
    if not live_mode:
        regs = DEMO_DATA
        st.info("📂 Demo data — toggle **Live Serial** in sidebar to poll hardware.")
    elif manual_poll or auto:
        with st.spinner(f"Polling {sel_port} @ {sel_baud} baud …"):
            regs = poll_input_bulk(sel_port, int(sel_baud), int(sel_slave))
        if not regs:
            st.error("❌ No response from inverter. Check cable, port and baud rate.")

    if regs:
        now = datetime.now()
        ts_str = now.strftime("%H:%M:%S")

        # ── Decode key values ─────────────────────────────────
        run_raw   = regs.get(0x0009, 0)
        grid_raw  = regs.get(0x001A, 0)
        bdc_raw   = regs.get(0x0019, 0)
        soc_raw   = regs.get(0x001C, 0)
        bms_raw   = regs.get(0x0017, 0)

        gv   = decode_reg(regs, 0x0000, "uint16", 0.1)
        gi   = decode_reg(regs, 0x0001, "int16",  0.1)
        gp   = decode_reg(regs, 0x0002, "int16",  1.0)
        gf   = decode_reg(regs, 0x0007, "uint16", 0.01)

        pv1v = decode_reg(regs, 0x0003, "uint16", 0.1)
        pv2v = decode_reg(regs, 0x0004, "uint16", 0.1)
        pv1i = decode_reg(regs, 0x0005, "uint16", 0.1)
        pv2i = decode_reg(regs, 0x0006, "uint16", 0.1)
        pv1p = decode_reg(regs, 0x000A, "uint16", 1.0) or 0
        pv2p = decode_reg(regs, 0x000B, "uint16", 1.0) or 0
        pv_total = pv1p + pv2p

        bv   = decode_reg(regs, 0x0014, "int16",  0.1)
        bi   = decode_reg(regs, 0x0015, "int16",  0.1)
        bp   = decode_reg(regs, 0x0016, "int16",  1.0) or 0
        bt   = decode_reg(regs, 0x0018, "int16",  1.0)
        soc  = soc_raw

        fi_lo = regs.get(0x0046, 0)
        fi_hi = regs.get(0x0047, 0)
        feedin = i32(fi_lo, fi_hi)

        inv_t = decode_reg(regs, 0x0008, "int16", 1.0)

        # ── History ──────────────────────────────────────────
        h = st.session_state.history
        h["ts"].append(ts_str)
        h["pv_power"].append(pv_total)
        h["bat_power"].append(bp)
        h["grid_power"].append(gp or 0)
        h["soc"].append(soc)

        # ── Status strip ─────────────────────────────────────
        section("System Status")
        s1, s2, s3, s4, s5, s6 = st.columns(6)
        s1.metric("Run Mode",    RUN_MODE.get(run_raw, f"?({run_raw})"))
        s2.metric("Grid",        GRID_STATUS.get(grid_raw, "?"))
        s3.metric("Battery",     BDC_STATUS.get(bdc_raw, "?"))
        s4.metric("Inv Temp",    f"{inv_t} °C" if inv_t is not None else "—")
        s5.metric("BMS",         BMS_CONN.get(bms_raw, "?"))
        s6.metric("Last Poll",   ts_str)

        st.divider()

        # ── PV ───────────────────────────────────────────────
        section("☀️  Solar PV  (X1 — two strings)")
        p1, p2, p3 = st.columns(3)
        with p1:
            st.markdown("**String 1**")
            pa, pb, pc = st.columns(3)
            pa.metric("Voltage",  f"{pv1v} V")
            pb.metric("Current",  f"{pv1i} A")
            pc.metric("Power",    f"{pv1p} W")
        with p2:
            st.markdown("**String 2**")
            pa, pb, pc = st.columns(3)
            pa.metric("Voltage",  f"{pv2v} V")
            pb.metric("Current",  f"{pv2i} A")
            pc.metric("Power",    f"{pv2p} W")
        with p3:
            st.markdown("**Combined**")
            st.metric("Total PV Power",  f"{pv_total} W")
            st.metric("Solar Today",
                      f"{decode_reg(regs, 0x0096, 'uint16', 0.1)} kWh")

        st.divider()

        # ── Battery ──────────────────────────────────────────
        section("🔋  Battery")
        b1, b2, b3, b4, b5 = st.columns(5)
        b1.metric("Voltage",     f"{bv} V")
        b2.metric("Current",     f"{bi} A",
                  help="Positive = charging, negative = discharging")
        b3.metric("Power",       f"{bp} W",
                  help="Positive = charging, negative = discharging")
        b4.metric("Temperature", f"{bt} °C")
        b5.metric("SOC",         f"{soc} %")
        soc_bar(soc)

        bc1, bc2 = st.columns(2)
        with bc1:
            bms_soc = decode_reg(regs, 0x00BE, "uint16", 1.0)
            bms_soh = decode_reg(regs, 0x00BF, "uint16", 1.0)
            cvh = decode_reg(regs, 0x00BC, "uint16", 0.001)
            cvl = decode_reg(regs, 0x00BD, "uint16", 0.001)
            st.metric("BMS SOC",         f"{bms_soc} %")
            st.metric("BMS SOH",         f"{bms_soh} %")
        with bc2:
            st.metric("Highest Cell V",  f"{cvh} V")
            st.metric("Lowest Cell V",   f"{cvl} V")
            spread = round((cvh or 0) - (cvl or 0), 3)
            st.metric("Cell Spread",     f"{spread} V",
                      delta="OK" if spread < 0.05 else "High",
                      delta_color="normal" if spread < 0.05 else "inverse")

        st.divider()

        # ── Grid / Feed-in ────────────────────────────────────
        section("🔌  Grid")
        g1, g2, g3, g4, g5 = st.columns(5)
        g1.metric("Voltage",      f"{gv} V")
        g2.metric("Current",      f"{gi} A")
        g3.metric("Power",        f"{gp} W")
        g4.metric("Frequency",    f"{gf} Hz")
        feedin_label = f"{'▲ Export' if feedin >= 0 else '▼ Import'} {abs(feedin)} W"
        g5.metric("Feed-in",      feedin_label)

        st.divider()

        # ── Energy Totals ─────────────────────────────────────
        section("⚡  Energy Today")
        e1, e2, e3, e4 = st.columns(4)
        e1.metric("Yield (AC port)",
                  f"{decode_reg(regs, 0x0050, 'uint16', 0.1)} kWh")
        e2.metric("Solar Generated",
                  f"{decode_reg(regs, 0x0096, 'uint16', 0.1)} kWh")
        e3.metric("Bat Charged",
                  f"{decode_reg(regs, 0x0091, 'uint16', 0.1)} kWh")
        e4.metric("Off-grid Yield",
                  f"{decode_reg(regs, 0x0090, 'uint16', 0.1)} kWh")

        st.divider()

        # ── Faults ───────────────────────────────────────────
        section("⚠️  Fault Status")
        inv_fault_word = (regs.get(0x0041, 0) << 16) | regs.get(0x0040, 0)
        bms_fault_word = (regs.get(0x0045, 0) << 16) | regs.get(0x0044, 0)
        pcs_fault = regs.get(0x003E, 0)
        mgr_fault = regs.get(0x0043, 0)

        fc1, fc2 = st.columns(2)
        with fc1:
            fault_expander("Inverter Faults (X1)", inv_fault_word, X1_FAULT_BITS)
        with fc2:
            fault_expander("BMS Warning", bms_fault_word, BMS_FAULT_BITS)

        pf1, pf2 = st.columns(2)
        with pf1:
            lv = "ok" if pcs_fault == 0 else "err"
            badge(f"PCS Fault: 0x{pcs_fault:04X}" if pcs_fault else "PCS: OK", lv)
        with pf2:
            lv = "ok" if mgr_fault == 0 else "err"
            badge(f"Manager Fault: 0x{mgr_fault:04X}" if mgr_fault else "Manager: OK", lv)

        st.divider()

        # ── Trend charts ─────────────────────────────────────
        if len(h["ts"]) > 1:
            section("📈  Trend  (last 60 polls)")
            chart_df = pd.DataFrame({
                "Time":       list(h["ts"]),
                "PV Power W": list(h["pv_power"]),
                "Bat Power W": list(h["bat_power"]),
                "Grid Power W": list(h["grid_power"]),
                "SOC %":      list(h["soc"]),
            }).set_index("Time")
            ch1, ch2 = st.columns(2)
            ch1.line_chart(chart_df[["PV Power W", "Bat Power W", "Grid Power W"]],
                           height=200)
            ch2.line_chart(chart_df[["SOC %"]], height=200)

        # ── Raw dump ─────────────────────────────────────────
        with st.expander("🗄  Raw Register Dump"):
            raw_df = pd.DataFrame([
                {"Addr": f"0x{k:04X}", "Dec": k, "uint16": v,
                 "int16": i16(v), "Hex": f"0x{v:04X}"}
                for k, v in sorted(regs.items())
            ])
            st.dataframe(raw_df, use_container_width=True, hide_index=True)

    else:
        st.info("Press **🔄 Poll Now** or enable **Auto-refresh** to fetch live data.")

# ╔═══════════════════════════════════════════════════════════╗
# ║  TAB 2 — DATA INTERPRETER                                ║
# ╚═══════════════════════════════════════════════════════════╝
with tab2:
    st.markdown("## 🔍 Data Interpreter")
    st.caption("Full register decode with proper scaling, units, and friendly labels. "
               "Works in File/Demo mode too.")

    di_c1, di_c2, di_c3 = st.columns([2, 1, 1])
    with di_c1:
        reg_type = st.radio("Register Bank",
                            ["Input Registers (0x04)", "Holding Registers (0x03)"],
                            horizontal=True)
    with di_c2:
        cat_filter = st.multiselect("Filter Categories", [], placeholder="All")
    with di_c3:
        st.markdown("<br>", unsafe_allow_html=True)
        do_interp = st.button("📡 Poll & Decode", use_container_width=True)

    fc_num  = 0x04 if "Input" in reg_type else 0x03
    params  = INPUT_REGS if fc_num == 0x04 else HOLDING_REGS

    # populate category filter dynamically
    all_cats = sorted(set(p[6] for p in params))
    cat_filter = st.multiselect("Category filter", all_cats,
                                default=all_cats, key="cat_f2",
                                label_visibility="collapsed")

    if do_interp:
        with st.spinner("Polling…"):
            if not live_mode:
                raw = DEMO_DATA if fc_num == 0x04 else {}
                if fc_num == 0x03:
                    st.info("Holding registers demo not available — connect hardware.")
            else:
                raw = (poll_input_bulk(sel_port, int(sel_baud), int(sel_slave))
                       if fc_num == 0x04
                       else poll_holding_bulk(sel_port, int(sel_baud), int(sel_slave)))

        if not raw and live_mode:
            st.error("No data received.")
        else:
            rows = []
            for p in params:
                addr, sname, desc, scale, unit, dtype, cat = p
                if cat not in cat_filter:
                    continue
                raw_v = raw.get(addr)
                dec   = decode_reg(raw, addr, dtype, scale) if raw_v is not None else None
                fv    = friendly(sname, dec, unit, raw_v) if raw_v is not None else "—"
                rows.append({
                    "Category":     cat,
                    "Address":      f"0x{addr:04X}",
                    "Name":         sname,
                    "Description":  desc,
                    "Raw (hex)":    f"0x{raw_v:04X}" if raw_v is not None else "—",
                    "Raw (dec)":    raw_v if raw_v is not None else "—",
                    "Scaled":       dec,
                    "Unit":         unit,
                    "Interpreted":  fv,
                    "Type":         dtype,
                })
            df_out = pd.DataFrame(rows)
            st.dataframe(
                df_out,
                use_container_width=True,
                hide_index=True,
                column_config={
                    "Address":     st.column_config.TextColumn(width=90),
                    "Name":        st.column_config.TextColumn(width=130),
                    "Scaled":      st.column_config.NumberColumn(width=100),
                    "Interpreted": st.column_config.TextColumn(width=200, label="Value"),
                },
            )
            st.download_button(
                "⬇️ Download CSV",
                df_out.to_csv(index=False),
                f"solax_x1_{datetime.now():%Y%m%d_%H%M%S}.csv",
                "text/csv",
            )
    else:
        # Show reference table
        st.info("Press **Poll & Decode** to read live data. Reference table shown below.")
        ref = [{"Cat": p[6], "Addr": f"0x{p[0]:04X}", "Name": p[1],
                "Description": p[2], "Scale": p[3], "Unit": p[4], "Type": p[5]}
               for p in params if p[6] in cat_filter]
        st.dataframe(pd.DataFrame(ref), use_container_width=True, hide_index=True)

# ╔═══════════════════════════════════════════════════════════╗
# ║  TAB 3 — MANUAL FRAME TEST                               ║
# ╚═══════════════════════════════════════════════════════════╝
with tab3:
    st.markdown("## 🛠 Manual Frame Test")
    st.caption("Build, preview and send any Modbus RTU frame. "
               "Equivalent to the `testFrames()` function in your original script.")

    if not live_mode:
        st.warning("Switch to **Live Serial** mode in the sidebar to send frames.")

    with st.expander("📖 Modbus RTU Frame Structure"):
        st.markdown("""
| Field | Size | Notes |
|---|---|---|
| Slave ID | 1 byte | Default `0x01` |
| Function Code | 1 byte | `0x03` Holding · `0x04` Input · `0x06` Write Single |
| Start Reg MSB | 1 byte | High byte of register address |
| Start Reg LSB | 1 byte | Low byte |
| Count / Value MSB | 1 byte | Number of regs to read · OR high byte of write value |
| Count / Value LSB | 1 byte | |
| CRC Low | 1 byte | Automatically computed (Modbus CRC-16) |
| CRC High | 1 byte | |

**Your script frame example** — read 2 regs starting at 0x0106:
`01 04 01 06 00 02` → CRC appended automatically
        """)

    mf_c1, mf_c2 = st.columns(2)
    with mf_c1:
        mf_fc = st.selectbox("Function Code", [
            "0x03 — Read Holding Register",
            "0x04 — Read Input Register",
            "0x06 — Write Single Register",
        ])
        mf_reg  = st.text_input("Start Register (hex)", value="0106",
                                 placeholder="e.g. 0046")
        mf_cnt  = st.number_input("Register Count (read) / Value (write)",
                                   min_value=0, max_value=65535, value=2)
    with mf_c2:
        mf_rpt  = st.number_input("Repeat", 1, 50, 1)
        mf_dly  = st.number_input("Delay between repeats (ms)", 100, 5000, 500)
        mf_hex_only = st.checkbox("Show hex only (no auto-decode)", value=False)

    # Build preview
    frame_preview = None
    try:
        fc_b  = int(mf_fc[:4], 16)
        reg_b = int(mf_reg.strip(), 16)
        frame_preview = build_frame(int(sel_slave), fc_b, reg_b, int(mf_cnt))
        st.code(f"Frame:  {frame_preview.hex(' ').upper()}", language="text")
    except Exception as e:
        st.warning(f"Frame build error: {e}")

    run_frames = st.button("🚀 Send Frame(s)", disabled=(frame_preview is None or not live_mode),
                            use_container_width=True)

    if run_frames and frame_preview and live_mode:
        log = []
        prog = st.progress(0)
        for i in range(int(mf_rpt)):
            try:
                fc_b   = int(mf_fc[:4], 16)
                reg_b  = int(mf_reg.strip(), 16)
                frame  = build_frame(int(sel_slave), fc_b, reg_b, int(mf_cnt))
                exp    = 5 + int(mf_cnt) * 2 if fc_b != 0x06 else 8
                _, resp = _send(sel_port, int(sel_baud), int(sel_slave),
                                fc_b, reg_b, int(mf_cnt), sel_timeout)
                if resp:
                    ok = crc_ok(resp, fc_b)
                    log.append({"#": i+1, "Sent": frame.hex(' ').upper(),
                                 "Response": resp.hex(' ').upper(),
                                 "CRC": "✅" if ok else "❌",
                                 "Bytes": len(resp)})
                else:
                    log.append({"#": i+1, "Sent": frame.hex(' ').upper(),
                                 "Response": "NO RESPONSE", "CRC": "—", "Bytes": 0})
            except Exception as ex:
                log.append({"#": i+1, "Sent": "ERROR",
                             "Response": str(ex), "CRC": "—", "Bytes": 0})
            prog.progress((i + 1) / int(mf_rpt))
            if i < int(mf_rpt) - 1:
                time.sleep(int(mf_dly) / 1000)

        st.dataframe(pd.DataFrame(log), use_container_width=True, hide_index=True)

        # Auto-decode responses
        if not mf_hex_only and fc_b in (0x03, 0x04):
            for entry in log:
                if "NO RESPONSE" not in entry["Response"] and "ERROR" not in entry["Response"]:
                    try:
                        raw_b = bytes.fromhex(entry["Response"].replace(" ", ""))
                        words = parse_words(raw_b)
                        dec_rows = []
                        for idx, w in enumerate(words):
                            addr = reg_b + idx
                            match = next(
                                ((n, d, s, u, t) for a, n, d, s, u, t, _ in
                                 (INPUT_REGS if fc_b == 0x04 else HOLDING_REGS)
                                 if a == addr), None)
                            row = {
                                "Offset": idx,
                                "Address": f"0x{addr:04X}",
                                "uint16": w,
                                "int16":  i16(w),
                                "Hex":    f"0x{w:04X}",
                                "×0.1":  round(w * 0.1, 1),
                                "×0.01": round(w * 0.01, 2),
                            }
                            if match:
                                n, d, s, u, t = match
                                row["Parameter"] = n
                                row["Scaled"] = f"{round(decode_reg({addr: w}, addr, t, s) or 0, 3)} {u}".strip()
                            else:
                                row["Parameter"] = "—"
                                row["Scaled"] = "—"
                            dec_rows.append(row)
                        st.caption(f"Decoded — Frame #{entry['#']}:")
                        st.dataframe(pd.DataFrame(dec_rows),
                                     use_container_width=True, hide_index=True)
                    except Exception as ex:
                        st.warning(f"Decode error (frame #{entry['#']}): {ex}")

# ╔═══════════════════════════════════════════════════════════╗
# ║  TAB 4 — HEX / EXCEL DECODER                             ║
# ╚═══════════════════════════════════════════════════════════╝
with tab4:
    st.markdown("## 🧩 Hex / Excel Decoder")
    st.caption("Decode captured frames without live hardware.")

    dec_mode = st.radio("Input type", ["Paste Hex", "Upload Excel"],
                         horizontal=True)

    # ── Paste Hex ─────────────────────────────────────────────
    if dec_mode == "Paste Hex":
        st.markdown("### Paste Raw Hex")

        with st.expander("📖 Example responses"):
            st.code("Read Input (Faults):       01 04 06 00 00 00 00 00 00 60 93")
            st.code("Read Holding (InverterSN): 01 03 0E 48 34 37 35 32 32 5A 48 45 4E 47 57 45 4E 63 26")
            st.code("Write Single (ack):        01 06 00 1F 00 00 48 0A")

        hex_in = st.text_area("Hex response (spaces/colons optional)",
                               placeholder="01 04 14 00 EB …",
                               height=90)
        hc1, hc2, hc3 = st.columns(3)
        with hc1:
            h_start = st.text_input("Start register (hex)", value="",
                                     placeholder="e.g. 0000")
        with hc2:
            h_fc_hint = st.selectbox("Function code hint",
                                     ["0x04 Input", "0x03 Holding", "0x06 Write / Auto"])
        with hc3:
            st.markdown("<br>", unsafe_allow_html=True)
            do_hex_dec = st.button("🔬 Decode", use_container_width=True)

        if do_hex_dec and hex_in.strip():
            clean = hex_in.replace(" ", "").replace(":", "").replace("-", "")
            try:
                raw_b = bytes.fromhex(clean)
            except ValueError:
                st.error("Invalid hex — check for typos.")
                st.stop()

            if len(raw_b) < 4:
                st.error("Frame too short.")
                st.stop()

            slave_r = raw_b[0]
            fc_r    = raw_b[1]
            fc_hint = int(h_fc_hint[:4], 16)
            crc_v   = crc_ok(raw_b, fc_r)

            # Frame summary
            st.markdown("#### Frame Summary")
            s1, s2, s3, s4 = st.columns(4)
            s1.metric("Slave ID", f"0x{slave_r:02X} ({slave_r})")
            s2.metric("Function Code", f"0x{fc_r:02X}")
            s3.metric("Total Bytes", len(raw_b))
            s4.metric("CRC", "✅ Valid" if crc_v else "❌ Invalid")

            if fc_r in (0x83, 0x84):
                st.error(f"Exception response — code: 0x{raw_b[2]:02X}")
            elif fc_r in (0x03, 0x04) and len(raw_b) > 3:
                byte_cnt = raw_b[2]
                words    = parse_words(raw_b)
                st.metric("Registers", len(words))

                # Determine start address
                try:
                    start_a = int(h_start.strip(), 16) if h_start.strip() else None
                except ValueError:
                    start_a = None

                param_ref = INPUT_REGS if fc_r == 0x04 else HOLDING_REGS
                param_lut = {p[0]: p for p in param_ref}

                dec_rows = []
                for idx, w in enumerate(words):
                    row = {
                        "Offset":  idx,
                        "uint16":  w,
                        "int16":   i16(w),
                        "Hex":     f"0x{w:04X}",
                        "×0.1":   round(w * 0.1, 1),
                        "×0.01":  round(w * 0.01, 2),
                    }
                    if start_a is not None:
                        addr = start_a + idx
                        row["Address"] = f"0x{addr:04X}"
                        if addr in param_lut:
                            p = param_lut[addr]
                            n, d, scale, unit, dtype = p[1], p[2], p[3], p[4], p[5]
                            dec = decode_reg({addr: w}, addr, dtype, scale)
                            row["Parameter"]   = n
                            row["Description"] = d
                            row["Scaled"]      = f"{dec} {unit}".strip()
                            row["Interpreted"] = friendly(n, dec, unit, w)
                    dec_rows.append(row)

                st.markdown("#### Register Values")
                st.dataframe(pd.DataFrame(dec_rows),
                             use_container_width=True, hide_index=True)

                # ASCII interpretation
                st.markdown("#### ASCII (for serial number / name registers)")
                try:
                    raw_chars = b""
                    for w in words:
                        raw_chars += bytes([(w >> 8) & 0xFF, w & 0xFF])
                    printable = "".join(
                        c if 32 <= ord(c) < 127 else "·"
                        for c in raw_chars.decode("latin-1")
                    )
                    st.code(printable)
                except Exception:
                    pass

            elif fc_r == 0x06:
                if len(raw_b) >= 6:
                    reg_a = (raw_b[2] << 8) | raw_b[3]
                    val_w = (raw_b[4] << 8) | raw_b[5]
                    st.markdown("#### Write Acknowledgement")
                    wa, wb = st.columns(2)
                    wa.metric("Register Written", f"0x{reg_a:04X} ({reg_a})")
                    wb.metric("Value Written", f"0x{val_w:04X} ({val_w})")

    # ── Upload Excel ──────────────────────────────────────────
    else:
        st.markdown("### Upload Inverter Response.xlsx")
        st.caption("Upload the Excel file produced by your script's `createFile()` function.")

        uploaded = st.file_uploader("Select file", type=["xlsx", "xls"])
        if uploaded:
            try:
                df_xl = pd.read_excel(uploaded)
            except Exception as e:
                st.error(f"Cannot read file: {e}")
                st.stop()

            st.markdown(f"**{len(df_xl)} rows loaded**")
            st.dataframe(df_xl.head(5), use_container_width=True, hide_index=True)

            if not {"Frame Sent", "Response"}.issubset(df_xl.columns):
                st.error("Expected columns: 'Frame Sent' and 'Response'")
                st.stop()

            xl_fc = st.radio("Assume function code in file",
                              ["0x04 Input", "0x03 Holding"], horizontal=True)
            xl_fc_num = 0x04 if "04" in xl_fc else 0x03
            param_ref = INPUT_REGS if xl_fc_num == 0x04 else HOLDING_REGS
            param_lut = {p[0]: p for p in param_ref}

            if st.button("🔬 Decode All Rows", use_container_width=True):
                all_rows = []
                errors   = []

                for row_idx, row in df_xl.iterrows():
                    sent_h = str(row["Frame Sent"]).replace(" ", "").replace("_", "")
                    resp_h = str(row["Response"]).replace(" ", "")

                    # Parse start register from sent frame
                    try:
                        sent_b  = bytes.fromhex(sent_h)
                        start_r = (sent_b[2] << 8) | sent_b[3]
                        fc_sent = sent_b[1]
                    except Exception:
                        errors.append(f"Row {row_idx}: bad sent frame '{sent_h}'")
                        continue

                    if resp_h.upper() in ("NORESPONSE", "NO_RESPONSE", "NONE", "NAN", ""):
                        all_rows.append({
                            "Row": row_idx, "Start Reg": f"0x{start_r:04X}",
                            "Parameter": "NO RESPONSE", "Scaled": "—", "Unit": "—",
                        })
                        continue

                    try:
                        raw_b = bytes.fromhex(resp_h)
                        words = parse_words(raw_b)
                    except Exception as ex:
                        errors.append(f"Row {row_idx}: parse error — {ex}")
                        continue

                    for idx, w in enumerate(words):
                        addr = start_r + idx
                        if addr in param_lut:
                            p = param_lut[addr]
                            n, d, scale, unit, dtype = p[1], p[2], p[3], p[4], p[5]
                            dec = decode_reg({addr: w}, addr, dtype, scale)
                            fv  = friendly(n, dec, unit, w)
                            all_rows.append({
                                "Row":        row_idx,
                                "Address":    f"0x{addr:04X}",
                                "Parameter":  n,
                                "Raw":        f"0x{w:04X} ({w})",
                                "Scaled":     dec,
                                "Unit":       unit,
                                "Interpreted": fv,
                                "Description": d,
                            })
                        else:
                            all_rows.append({
                                "Row":       row_idx,
                                "Address":   f"0x{addr:04X}",
                                "Parameter": "—",
                                "Raw":       f"0x{w:04X} ({w})",
                                "Scaled":    w,
                                "Unit":      "",
                                "Interpreted": str(w),
                                "Description": "",
                            })

                if errors:
                    with st.expander(f"⚠️ {len(errors)} parsing errors"):
                        for e in errors:
                            st.caption(e)

                if all_rows:
                    df_decoded = pd.DataFrame(all_rows)
                    st.markdown(f"#### Decoded — {len(df_decoded)} register values")
                    st.dataframe(df_decoded, use_container_width=True, hide_index=True)
                    st.download_button(
                        "⬇️ Download Decoded CSV",
                        df_decoded.to_csv(index=False),
                        f"solax_x1_decoded_{datetime.now():%Y%m%d_%H%M%S}.csv",
                        "text/csv",
                    )

# ═════════════════════════════════════════════════════════════
# FOOTER
# ═════════════════════════════════════════════════════════════
st.divider()
st.caption(
    "⚡ SolaX X1 Hybrid G4 · Modbus RTU V3.21 · Single-phase  |  "
    "⚠️ EEprom registers (★) have limited write cycles — use Write Single sparingly  |  "
    "Min frame interval: **1 s**  ·  Inter-packet gap: **>100 ms**"
)