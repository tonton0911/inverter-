"""
Modbus RTU Inspector
====================
Generic Modbus RTU frame builder, sender, and response decoder.
Inverter-specific register maps are loaded as optional profiles.
Profiles expose only electrical telemetry: voltage, current, power,
frequency, power factor, and temperature.

Adding a new inverter: define an InverterProfile and add it to PROFILES.

Run:
  pip install -r requirements.txt
  streamlit run app.py
"""
from __future__ import annotations

import logging
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from typing import Callable, Optional

import crcmod
import pandas as pd
import streamlit as st

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

log = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════════════════
# MODBUS CORE  —  generic, no inverter knowledge
# ═══════════════════════════════════════════════════════════════════════════

_crc16 = crcmod.predefined.mkCrcFun("modbus")

FC_READ_HOLDING = 0x03
FC_READ_INPUT   = 0x04
FC_WRITE_SINGLE = 0x06

VALID_BAUDS = (9600, 19200, 38400, 57600, 115200)


def build_frame(slave: int, fc: int, register: int, value: int) -> bytes:
    body = bytes([
        slave, fc,
        (register >> 8) & 0xFF, register & 0xFF,
        (value   >> 8) & 0xFF, value   & 0xFF,
    ])
    crc = _crc16(body)
    return body + bytes([crc & 0xFF, crc >> 8])


def crc_valid(frame: bytes, expected_fc: int) -> bool:
    if len(frame) < 4 or frame[1] != expected_fc:
        return False
    calc = _crc16(frame[:-2])
    recv = frame[-2] | (frame[-1] << 8)
    return calc == recv


def parse_words(response: bytes) -> list[int]:
    if len(response) < 5:
        return []
    n = response[2]
    payload = response[3: 3 + n]
    return [(payload[i] << 8) | payload[i + 1] for i in range(0, len(payload) - 1, 2)]


def send_frame(
    port: str,
    baud: int,
    slave: int,
    fc: int,
    register: int,
    value: int,
    *,
    timeout: float = 1.0,
    retries: int = 2,
    gap: float = 0.12,
) -> tuple[bytes, Optional[bytes]]:
    """Send one Modbus RTU request, return (frame_sent, response | None)."""
    frame    = build_frame(slave, fc, register, value)
    expected = 5 + value * 2 if fc != FC_WRITE_SINGLE else 8

    for attempt in range(1 + retries):
        try:
            with serial.Serial(port, baud, timeout=timeout) as conn:
                time.sleep(gap)
                conn.write(frame)
                resp = conn.read(expected)
            if resp and crc_valid(resp, fc):
                return frame, resp
            log.debug("attempt %d: bad/empty response for reg 0x%04X (%d bytes)",
                      attempt + 1, register, len(resp) if resp else 0)
        except Exception as exc:
            log.warning("serial error attempt %d: %s", attempt + 1, exc)
        if attempt < retries:
            time.sleep(0.2)

    log.error("no valid response for reg 0x%04X after %d attempts", register, 1 + retries)
    return frame, None


def as_int16(v: int) -> int:
    return v if v < 0x8000 else v - 0x10000


def as_int32(lo: int, hi: int) -> int:
    c = (hi << 16) | lo
    return c if c < 0x80000000 else c - 0x100000000


def poll_chunks(
    port: str, baud: int, slave: int,
    fc: int, chunks: list[tuple[int, int]],
    timeout: float, retries: int, gap: float,
) -> dict[int, int]:
    result: dict[int, int] = {}
    for start, count in chunks:
        _, resp = send_frame(port, baud, slave, fc, start, count,
                             timeout=timeout, retries=retries, gap=gap)
        if resp:
            for i, w in enumerate(parse_words(resp)):
                result[start + i] = w
        else:
            log.warning("no data for chunk 0x%04X (fc=0x%02X)", start, fc)
        time.sleep(gap)
    return result


# ═══════════════════════════════════════════════════════════════════════════
# INVERTER PROFILE SYSTEM
# ═══════════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class RegDef:
    addr:     int
    name:     str
    desc:     str
    scale:    float
    unit:     str
    dtype:    str        # uint16 | int16 | int32 | ascii
    category: str
    lookup:   Optional[str] = None


@dataclass
class InverterProfile:
    """Self-contained inverter definition. Add one per brand."""
    name:           str
    input_regs:     dict[int, RegDef]         = field(default_factory=dict)
    holding_regs:   dict[int, RegDef]         = field(default_factory=dict)
    input_chunks:   list[tuple[int, int]]     = field(default_factory=list)
    holding_chunks: list[tuple[int, int]]     = field(default_factory=list)
    lookups:        dict[str, dict[int, str]] = field(default_factory=dict)
    demo_words:     dict[int, int]            = field(default_factory=dict)
    build_snapshot: Optional[Callable]        = field(default=None, compare=False)


def decode_reg(word_map: dict[int, int], reg: RegDef) -> Optional[float]:
    if reg.addr not in word_map:
        return None
    raw = word_map[reg.addr]
    if reg.dtype == "int16":
        value = as_int16(raw)
    elif reg.dtype == "int32":
        value = as_int32(raw, word_map.get(reg.addr + 1, 0))
    else:
        value = raw
    return round(value * reg.scale, 3)


def resolve_label(profile: InverterProfile, reg: RegDef, raw: int) -> Optional[str]:
    if reg.lookup and reg.lookup in profile.lookups:
        result = profile.lookups[reg.lookup].get(raw)
        return str(result) if result is not None else None
    return None


# ── Shared snapshot builder helper ────────────────────────────────────────
# All profiles use this same structure. Each profile passes its own
# address mapping via snap_map: {snapshot_key: (addr, dtype, scale)}.

def _build_generic_snapshot(
    words: dict[int, int],
    snap_map: dict[str, tuple[int, str, float]],
    status_keys: dict[str, int],    # snapshot_key -> raw addr (int values, no scale)
) -> dict:
    def val(addr: int, dtype: str, scale: float) -> Optional[float]:
        if addr not in words:
            return None
        raw = words[addr]
        if dtype == "int16":
            raw = as_int16(raw)
        elif dtype == "int32":
            raw = as_int32(raw, words.get(addr + 1, 0))
        return round(raw * scale, 3)

    snap: dict = {"timestamp": datetime.now(), "_words": words}
    for key, (addr, dtype, scale) in snap_map.items():
        snap[key] = val(addr, dtype, scale)
    for key, addr in status_keys.items():
        snap[key] = words.get(addr, 0)

    pv1 = snap.get("pv1_power") or 0
    pv2 = snap.get("pv2_power") or 0
    snap["pv_total"] = round(pv1 + pv2, 1)
    return snap


# ═══════════════════════════════════════════════════════════════════════════
# PROFILE HELPERS
# ═══════════════════════════════════════════════════════════════════════════

def _r(addr, name, desc, scale, unit, dtype, cat, lookup=None):
    return RegDef(addr, name, desc, scale, unit, dtype, cat, lookup)


# ═══════════════════════════════════════════════════════════════════════════
# SOLAX X1 HYBRID G4  —  Protocol V3.21
# ═══════════════════════════════════════════════════════════════════════════

_SOLAX_INPUT = [
    _r(0x0000, "grid_voltage",   "Grid RMS Voltage",          0.1,  "V",  "uint16", "grid"),
    _r(0x0001, "grid_current",   "Grid RMS Current",          0.1,  "A",  "int16",  "grid"),
    _r(0x0002, "grid_power",     "Grid Active Power",         1.0,  "W",  "int16",  "grid"),
    _r(0x0007, "grid_frequency", "Grid Frequency",            0.01, "Hz", "uint16", "grid"),
    _r(0x001A, "grid_status",    "On-Grid / Off-Grid",        1.0,  "",   "uint16", "grid", "grid_status"),
    _r(0x0003, "pv1_voltage",    "PV String 1 Voltage",       0.1,  "V",  "uint16", "pv"),
    _r(0x0005, "pv1_current",    "PV String 1 Current",       0.1,  "A",  "uint16", "pv"),
    _r(0x000A, "pv1_power",      "DC Power String 1",         1.0,  "W",  "uint16", "pv"),
    _r(0x0004, "pv2_voltage",    "PV String 2 Voltage",       0.1,  "V",  "uint16", "pv"),
    _r(0x0006, "pv2_current",    "PV String 2 Current",       0.1,  "A",  "uint16", "pv"),
    _r(0x000B, "pv2_power",      "DC Power String 2",         1.0,  "W",  "uint16", "pv"),
    _r(0x0008, "inv_temp",       "Inverter Temperature",      1.0,  "°C", "int16",  "inverter"),
    _r(0x0009, "run_mode",       "Inverter Run Mode",         1.0,  "",   "uint16", "inverter", "run_mode"),
    _r(0x0014, "bat_voltage",    "Battery Voltage",           0.1,  "V",  "int16",  "battery"),
    _r(0x0015, "bat_current",    "Battery Current",           0.1,  "A",  "int16",  "battery"),
    _r(0x0016, "bat_power",      "Battery Power",             1.0,  "W",  "int16",  "battery"),
    _r(0x0018, "bat_temp",       "Battery Temperature",       1.0,  "°C", "int16",  "battery"),
    _r(0x001C, "soc",            "State of Charge",           1.0,  "%",  "uint16", "battery"),
    _r(0x0019, "bdc_status",     "Battery Direction",         1.0,  "",   "uint16", "battery", "bdc_status"),
    _r(0x0046, "feedin_power",   "Feed-in Power",             1.0,  "W",  "int32",  "grid"),
]

_SOLAX_LOOKUPS = {
    "run_mode": {
        0: "Waiting", 1: "Checking", 2: "Normal",  3: "Fault",
        4: "Perm Fault", 5: "Update", 6: "OG Wait", 7: "Off-grid",
        8: "Self-test", 9: "Idle", 10: "Standby",
    },
    "bdc_status":  {0: "Discharging", 1: "Charging", 2: "Stopped"},
    "grid_status": {0: "On-Grid", 1: "Off-Grid"},
}

_SOLAX_DEMO: dict[int, int] = {
    0x0000: 2300, 0x0001: 150,  0x0002: 345,  0x0007: 5001,
    0x001A: 0,    0x0003: 3850, 0x0005: 85,   0x000A: 3270,
    0x0004: 3720, 0x0006: 60,   0x000B: 2232, 0x0008: 42,
    0x0009: 2,    0x0014: 520,  0x0015: 80,   0x0016: 416,
    0x0018: 28,   0x001C: 72,   0x0019: 1,
    0x0046: 0xFEE0, 0x0047: 0xFFFF,  # -288 W
}

_SOLAX_SNAP_MAP = {
    "grid_voltage":   (0x0000, "uint16", 0.1),
    "grid_current":   (0x0001, "int16",  0.1),
    "grid_power":     (0x0002, "int16",  1.0),
    "grid_frequency": (0x0007, "uint16", 0.01),
    "feedin_power":   (0x0046, "int32",  1.0),
    "pv1_voltage":    (0x0003, "uint16", 0.1),
    "pv1_current":    (0x0005, "uint16", 0.1),
    "pv1_power":      (0x000A, "uint16", 1.0),
    "pv2_voltage":    (0x0004, "uint16", 0.1),
    "pv2_current":    (0x0006, "uint16", 0.1),
    "pv2_power":      (0x000B, "uint16", 1.0),
    "inv_temp":       (0x0008, "int16",  1.0),
    "bat_voltage":    (0x0014, "int16",  0.1),
    "bat_current":    (0x0015, "int16",  0.1),
    "bat_power":      (0x0016, "int16",  1.0),
    "bat_temp":       (0x0018, "int16",  1.0),
    "soc":            (0x001C, "uint16", 1.0),
}
_SOLAX_STATUS = {"run_mode": 0x0009, "grid_status": 0x001A, "bdc_status": 0x0019}

SOLAX_X1 = InverterProfile(
    name          = "SolaX X1 Hybrid G4",
    input_regs    = {r.addr: r for r in _SOLAX_INPUT},
    input_chunks  = [
        (0x0000, 0x001D), (0x0046, 0x0002),
    ],
    lookups       = _SOLAX_LOOKUPS,
    demo_words    = _SOLAX_DEMO,
    build_snapshot= lambda w: _build_generic_snapshot(w, _SOLAX_SNAP_MAP, _SOLAX_STATUS),
)


# ═══════════════════════════════════════════════════════════════════════════
# HUAWEI SUN2000 SERIES  —  Modbus TCP/RTU (SUN2000-*KTL)
# Ref: SUN2000 Modbus Interface Definition (SmartLogger)
# ═══════════════════════════════════════════════════════════════════════════

_HUAWEI_INPUT = [
    _r(0x2000, "grid_voltage_ab",  "Line Voltage A-B",         0.1,  "V",   "uint16", "grid"),
    _r(0x2001, "grid_voltage_bc",  "Line Voltage B-C",         0.1,  "V",   "uint16", "grid"),
    _r(0x2002, "grid_voltage_ca",  "Line Voltage C-A",         0.1,  "V",   "uint16", "grid"),
    _r(0x2003, "grid_voltage_a",   "Phase A Voltage",          0.1,  "V",   "uint16", "grid"),
    _r(0x2004, "grid_voltage_b",   "Phase B Voltage",          0.1,  "V",   "uint16", "grid"),
    _r(0x2005, "grid_voltage_c",   "Phase C Voltage",          0.1,  "V",   "uint16", "grid"),
    _r(0x2006, "grid_current_a",   "Phase A Current",          0.001,"A",   "int32",  "grid"),
    _r(0x2008, "grid_current_b",   "Phase B Current",          0.001,"A",   "int32",  "grid"),
    _r(0x200A, "grid_current_c",   "Phase C Current",          0.001,"A",   "int32",  "grid"),
    _r(0x200E, "grid_power",       "Active Power",             1.0,  "W",   "int32",  "grid"),
    _r(0x2010, "reactive_power",   "Reactive Power",           1.0,  "var", "int32",  "grid"),
    _r(0x2012, "power_factor",     "Power Factor",             0.001,"",    "int16",  "grid"),
    _r(0x2013, "grid_frequency",   "Grid Frequency",           0.01, "Hz",  "uint16", "grid"),
    _r(0x2014, "efficiency",       "Inverter Efficiency",      0.01, "%",   "uint16", "grid"),
    _r(0x2015, "inv_temp",         "Internal Temperature",     0.1,  "°C",  "int16",  "inverter"),
    _r(0x2016, "inv_temp2",        "Heat Sink Temperature",    0.1,  "°C",  "int16",  "inverter"),
    _r(0x2017, "run_mode",         "Device Status",            1.0,  "",    "uint16", "inverter", "run_mode"),
    _r(0x2027, "pv1_voltage",      "PV1 Voltage",              0.1,  "V",   "uint16", "pv"),
    _r(0x2028, "pv1_current",      "PV1 Current",              0.01, "A",   "int16",  "pv"),
    _r(0x2029, "pv2_voltage",      "PV2 Voltage",              0.1,  "V",   "uint16", "pv"),
    _r(0x202A, "pv2_current",      "PV2 Current",              0.01, "A",   "int16",  "pv"),
    _r(0x202B, "pv1_power",        "PV1 Power",                1.0,  "W",   "int32",  "pv"),
    _r(0x202D, "pv2_power",        "PV2 Power",                1.0,  "W",   "int32",  "pv"),
    _r(0x101C, "bat_voltage",      "Battery Voltage",          0.1,  "V",   "int16",  "battery"),
    _r(0x101D, "bat_current",      "Battery Current",          0.1,  "A",   "int16",  "battery"),
    _r(0x101E, "bat_power",        "Charge/Discharge Power",   1.0,  "W",   "int32",  "battery"),
    _r(0x1020, "bat_temp",         "Battery Temperature",      0.1,  "°C",  "int16",  "battery"),
    _r(0x1025, "soc",              "State of Charge",          0.1,  "%",   "uint16", "battery"),
    _r(0x1000, "bdc_status",       "Battery Working Mode",     1.0,  "",    "uint16", "battery", "bdc_status"),
]

_HUAWEI_LOOKUPS = {
    "run_mode": {
        0: "Standby", 512: "Initialising", 1024: "Grid-connected",
        1025: "Grid-connected (normal)", 1026: "Grid-conn (derating)",
        1280: "Shutdown", 1536: "Fault",
    },
    "bdc_status": {0: "Offline", 1: "Standby", 2: "Running", 3: "Fault", 4: "Sleep"},
}

_HUAWEI_DEMO: dict[int, int] = {
    0x2000: 4000, 0x2001: 4002, 0x2002: 3998,
    0x2003: 2310, 0x2004: 2308, 0x2005: 2312,
    0x2006: 150,  0x2007: 0,    0x2008: 148,  0x2009: 0,
    0x200A: 151,  0x200B: 0,
    0x200E: 3400, 0x200F: 0,
    0x2010: 200,  0x2011: 0,
    0x2012: 998,  0x2013: 5000, 0x2014: 975,
    0x2015: 410,  0x2016: 385,  0x2017: 1025,
    0x2027: 3850, 0x2028: 88,   0x2029: 3720, 0x202A: 62,
    0x202B: 3381, 0x202C: 0,    0x202D: 2306, 0x202E: 0,
    0x101C: 520,  0x101D: 80,   0x101E: 416,  0x101F: 0,
    0x1020: 280,  0x1025: 720,  0x1000: 2,
}

_HUAWEI_SNAP_MAP = {
    "grid_voltage":   (0x2003, "uint16", 0.1),
    "grid_current":   (0x2006, "int32",  0.001),
    "grid_power":     (0x200E, "int32",  1.0),
    "grid_frequency": (0x2013, "uint16", 0.01),
    "power_factor":   (0x2012, "int16",  0.001),
    "inv_temp":       (0x2015, "int16",  0.1),
    "pv1_voltage":    (0x2027, "uint16", 0.1),
    "pv1_current":    (0x2028, "int16",  0.01),
    "pv1_power":      (0x202B, "int32",  1.0),
    "pv2_voltage":    (0x2029, "uint16", 0.1),
    "pv2_current":    (0x202A, "int16",  0.01),
    "pv2_power":      (0x202D, "int32",  1.0),
    "bat_voltage":    (0x101C, "int16",  0.1),
    "bat_current":    (0x101D, "int16",  0.1),
    "bat_power":      (0x101E, "int32",  1.0),
    "bat_temp":       (0x1020, "int16",  0.1),
    "soc":            (0x1025, "uint16", 0.1),
}
_HUAWEI_STATUS = {"run_mode": 0x2017, "bdc_status": 0x1000}

HUAWEI_SUN2000 = InverterProfile(
    name          = "Huawei SUN2000",
    input_regs    = {r.addr: r for r in _HUAWEI_INPUT},
    input_chunks  = [
        (0x1000, 0x0026), (0x2000, 0x0030),
    ],
    lookups       = _HUAWEI_LOOKUPS,
    demo_words    = _HUAWEI_DEMO,
    build_snapshot= lambda w: _build_generic_snapshot(w, _HUAWEI_SNAP_MAP, _HUAWEI_STATUS),
)


# ═══════════════════════════════════════════════════════════════════════════
# DEYE SUN-*K-SG04LP / SG01HP  —  Modbus RTU
# Ref: Deye Inverter Modbus Protocol (public v1.34)
# ═══════════════════════════════════════════════════════════════════════════

_DEYE_INPUT = [
    _r(0x006D, "grid_voltage_r",  "Grid Voltage Phase R",      0.1,  "V",   "uint16", "grid"),
    _r(0x006E, "grid_voltage_s",  "Grid Voltage Phase S",      0.1,  "V",   "uint16", "grid"),
    _r(0x006F, "grid_voltage_t",  "Grid Voltage Phase T",      0.1,  "V",   "uint16", "grid"),
    _r(0x0076, "grid_current_r",  "Grid Current Phase R",      0.01, "A",   "int16",  "grid"),
    _r(0x0077, "grid_current_s",  "Grid Current Phase S",      0.01, "A",   "int16",  "grid"),
    _r(0x0078, "grid_current_t",  "Grid Current Phase T",      0.01, "A",   "int16",  "grid"),
    _r(0x0079, "grid_power",      "Total Grid Active Power",   1.0,  "W",   "int16",  "grid"),
    _r(0x007F, "grid_frequency",  "Grid Frequency",            0.01, "Hz",  "uint16", "grid"),
    _r(0x0082, "power_factor",    "Power Factor",              0.001,"",    "int16",  "grid"),
    _r(0x0090, "inv_temp",        "DC Temperature",            0.1,  "°C",  "int16",  "inverter"),
    _r(0x0091, "inv_temp2",       "AC Temperature",            0.1,  "°C",  "int16",  "inverter"),
    _r(0x0096, "run_mode",        "Running Status",            1.0,  "",    "uint16", "inverter", "run_mode"),
    _r(0x00BA, "pv1_voltage",     "PV1 Voltage",               0.1,  "V",   "uint16", "pv"),
    _r(0x00BB, "pv1_current",     "PV1 Current",               0.01, "A",   "uint16", "pv"),
    _r(0x00BC, "pv1_power",       "PV1 Power",                 1.0,  "W",   "uint16", "pv"),
    _r(0x00BD, "pv2_voltage",     "PV2 Voltage",               0.1,  "V",   "uint16", "pv"),
    _r(0x00BE, "pv2_current",     "PV2 Current",               0.01, "A",   "uint16", "pv"),
    _r(0x00BF, "pv2_power",       "PV2 Power",                 1.0,  "W",   "uint16", "pv"),
    _r(0x00B8, "bat_voltage",     "Battery Voltage",           0.01, "V",   "uint16", "battery"),
    _r(0x00B9, "bat_current",     "Battery Current",           0.01, "A",   "int16",  "battery"),
    _r(0x00BE, "bat_power",       "Battery Power",             1.0,  "W",   "int16",  "battery"),
    _r(0x00B5, "bat_temp",        "Battery Temperature",       0.1,  "°C",  "int16",  "battery"),
    _r(0x00B6, "soc",             "Battery SOC",               1.0,  "%",   "uint16", "battery"),
    _r(0x00B3, "bdc_status",      "Battery Status",            1.0,  "",    "uint16", "battery", "bdc_status"),
]

_DEYE_LOOKUPS = {
    "run_mode": {
        0: "Standby", 1: "Self-test", 2: "Normal", 3: "Alarm", 4: "Fault",
    },
    "bdc_status": {0: "Discharging", 1: "Charging", 2: "Standby"},
}

_DEYE_DEMO: dict[int, int] = {
    0x006D: 2310, 0x006E: 2308, 0x006F: 2312,
    0x0076: 148,  0x0077: 146,  0x0078: 150,
    0x0079: 3400, 0x007F: 5001, 0x0082: 998,
    0x0090: 410,  0x0091: 385,  0x0096: 2,
    0x00BA: 3850, 0x00BB: 88,   0x00BC: 3381,
    0x00BD: 3720, 0x00BE: 62,   0x00BF: 2306,
    0x00B8: 5200, 0x00B9: 80,   0x00B5: 280,
    0x00B6: 72,   0x00B3: 1,
}

_DEYE_SNAP_MAP = {
    "grid_voltage":   (0x006D, "uint16", 0.1),
    "grid_current":   (0x0076, "int16",  0.01),
    "grid_power":     (0x0079, "int16",  1.0),
    "grid_frequency": (0x007F, "uint16", 0.01),
    "power_factor":   (0x0082, "int16",  0.001),
    "inv_temp":       (0x0090, "int16",  0.1),
    "pv1_voltage":    (0x00BA, "uint16", 0.1),
    "pv1_current":    (0x00BB, "uint16", 0.01),
    "pv1_power":      (0x00BC, "uint16", 1.0),
    "pv2_voltage":    (0x00BD, "uint16", 0.1),
    "pv2_current":    (0x00BE, "uint16", 0.01),
    "pv2_power":      (0x00BF, "uint16", 1.0),
    "bat_voltage":    (0x00B8, "uint16", 0.01),
    "bat_current":    (0x00B9, "int16",  0.01),
    "bat_temp":       (0x00B5, "int16",  0.1),
    "soc":            (0x00B6, "uint16", 1.0),
}
_DEYE_STATUS = {"run_mode": 0x0096, "bdc_status": 0x00B3}

DEYE_SUN = InverterProfile(
    name          = "Deye SUN Series",
    input_regs    = {r.addr: r for r in _DEYE_INPUT},
    input_chunks  = [
        (0x006D, 0x0020), (0x00B3, 0x0010),
    ],
    lookups       = _DEYE_LOOKUPS,
    demo_words    = _DEYE_DEMO,
    build_snapshot= lambda w: _build_generic_snapshot(w, _DEYE_SNAP_MAP, _DEYE_STATUS),
)


# ═══════════════════════════════════════════════════════════════════════════
# SOLIS S5/S6 SERIES  —  Modbus RTU
# Ref: Solis Modbus Communication Protocol V1.8
# ═══════════════════════════════════════════════════════════════════════════

_SOLIS_INPUT = [
    _r(0x0033, "pv1_voltage",    "DC Voltage 1",             0.1,  "V",   "uint16", "pv"),
    _r(0x0034, "pv1_current",    "DC Current 1",             0.1,  "A",   "uint16", "pv"),
    _r(0x0035, "pv1_power",      "DC Power 1",               1.0,  "W",   "uint32", "pv"),
    _r(0x0037, "pv2_voltage",    "DC Voltage 2",             0.1,  "V",   "uint16", "pv"),
    _r(0x0038, "pv2_current",    "DC Current 2",             0.1,  "A",   "uint16", "pv"),
    _r(0x0039, "pv2_power",      "DC Power 2",               1.0,  "W",   "uint32", "pv"),
    _r(0x004A, "inv_temp",       "Inverter Temperature",     0.1,  "°C",  "int16",  "inverter"),
    _r(0x004D, "grid_frequency", "Grid Frequency",           0.01, "Hz",  "uint16", "grid"),
    _r(0x004E, "grid_power",     "Active Power Output",      1.0,  "W",   "int32",  "grid"),
    _r(0x0050, "reactive_power", "Reactive Power Output",    1.0,  "var", "int32",  "grid"),
    _r(0x004C, "power_factor",   "Power Factor",             0.001,"",    "int16",  "grid"),
    _r(0x0055, "grid_voltage_a", "Phase A Voltage",          0.1,  "V",   "uint16", "grid"),
    _r(0x0056, "grid_current_a", "Phase A Current",          0.1,  "A",   "int16",  "grid"),
    _r(0x0057, "grid_voltage_b", "Phase B Voltage",          0.1,  "V",   "uint16", "grid"),
    _r(0x0058, "grid_current_b", "Phase B Current",          0.1,  "A",   "int16",  "grid"),
    _r(0x0059, "grid_voltage_c", "Phase C Voltage",          0.1,  "V",   "uint16", "grid"),
    _r(0x005A, "grid_current_c", "Phase C Current",          0.1,  "A",   "int16",  "grid"),
    _r(0x00A0, "bat_voltage",    "Battery Voltage",          0.01, "V",   "uint16", "battery"),
    _r(0x00A1, "bat_current",    "Battery Current",          0.01, "A",   "int16",  "battery"),
    _r(0x00A2, "bat_power",      "Battery Power",            1.0,  "W",   "int32",  "battery"),
    _r(0x00A7, "bat_temp",       "Battery Temperature",      0.1,  "°C",  "int16",  "battery"),
    _r(0x00A9, "soc",            "Battery SOC",              0.1,  "%",   "uint16", "battery"),
    _r(0x009B, "run_mode",       "Inverter Status",          1.0,  "",    "uint16", "inverter", "run_mode"),
    _r(0x00AB, "bdc_status",     "Battery Status",           1.0,  "",    "uint16", "battery",  "bdc_status"),
]

_SOLIS_LOOKUPS = {
    "run_mode": {
        0: "Waiting",  1: "OpenRun",  2: "SoftRun", 3: "Normal",
        4: "Grid Off", 5: "Fault",    6: "Standby",
    },
    "bdc_status": {0: "Not charging", 1: "Charging", 2: "Discharging", 3: "Standby"},
}

_SOLIS_DEMO: dict[int, int] = {
    0x0033: 3850, 0x0034: 85,   0x0035: 3270, 0x0036: 0,
    0x0037: 3720, 0x0038: 60,   0x0039: 2232, 0x003A: 0,
    0x004A: 420,  0x004D: 5001,
    0x004E: 3400, 0x004F: 0,    0x0050: 200,  0x0051: 0,
    0x004C: 998,
    0x0055: 2310, 0x0056: 148,
    0x0057: 2308, 0x0058: 146,
    0x0059: 2312, 0x005A: 150,
    0x00A0: 5200, 0x00A1: 80,   0x00A2: 416,  0x00A3: 0,
    0x00A7: 280,  0x00A9: 720,  0x009B: 3,    0x00AB: 1,
}

_SOLIS_SNAP_MAP = {
    "grid_voltage":   (0x0055, "uint16", 0.1),
    "grid_current":   (0x0056, "int16",  0.1),
    "grid_power":     (0x004E, "int32",  1.0),
    "grid_frequency": (0x004D, "uint16", 0.01),
    "power_factor":   (0x004C, "int16",  0.001),
    "inv_temp":       (0x004A, "int16",  0.1),
    "pv1_voltage":    (0x0033, "uint16", 0.1),
    "pv1_current":    (0x0034, "uint16", 0.1),
    "pv1_power":      (0x0035, "uint32", 1.0),
    "pv2_voltage":    (0x0037, "uint16", 0.1),
    "pv2_current":    (0x0038, "uint16", 0.1),
    "pv2_power":      (0x0039, "uint32", 1.0),
    "bat_voltage":    (0x00A0, "uint16", 0.01),
    "bat_current":    (0x00A1, "int16",  0.01),
    "bat_power":      (0x00A2, "int32",  1.0),
    "bat_temp":       (0x00A7, "int16",  0.1),
    "soc":            (0x00A9, "uint16", 0.1),
}
_SOLIS_STATUS = {"run_mode": 0x009B, "bdc_status": 0x00AB}

SOLIS = InverterProfile(
    name          = "Solis S5/S6",
    input_regs    = {r.addr: r for r in _SOLIS_INPUT},
    input_chunks  = [
        (0x0033, 0x0028), (0x009B, 0x0015),
    ],
    lookups       = _SOLIS_LOOKUPS,
    demo_words    = _SOLIS_DEMO,
    build_snapshot= lambda w: _build_generic_snapshot(w, _SOLIS_SNAP_MAP, _SOLIS_STATUS),
)


# ═══════════════════════════════════════════════════════════════════════════
# SAJ H1/R5/HS2  —  Modbus RTU
# Ref: SAJ Inverter Modbus Protocol V1.0
# ═══════════════════════════════════════════════════════════════════════════

_SAJ_INPUT = [
    _r(0x0100, "run_mode",       "Device State",              1.0,  "",    "uint16", "inverter", "run_mode"),
    _r(0x0101, "grid_voltage_a", "Grid Voltage L1",           0.1,  "V",   "uint16", "grid"),
    _r(0x0102, "grid_voltage_b", "Grid Voltage L2",           0.1,  "V",   "uint16", "grid"),
    _r(0x0103, "grid_voltage_c", "Grid Voltage L3",           0.1,  "V",   "uint16", "grid"),
    _r(0x0104, "grid_current_a", "Grid Current L1",           0.01, "A",   "int16",  "grid"),
    _r(0x0105, "grid_current_b", "Grid Current L2",           0.01, "A",   "int16",  "grid"),
    _r(0x0106, "grid_current_c", "Grid Current L3",           0.01, "A",   "int16",  "grid"),
    _r(0x0107, "grid_power",     "Total Active Power",        1.0,  "W",   "int32",  "grid"),
    _r(0x0109, "reactive_power", "Total Reactive Power",      1.0,  "var", "int32",  "grid"),
    _r(0x010B, "power_factor",   "Power Factor",              0.001,"",    "int16",  "grid"),
    _r(0x010C, "grid_frequency", "Grid Frequency",            0.01, "Hz",  "uint16", "grid"),
    _r(0x0115, "inv_temp",       "Inverter Temperature",      0.1,  "°C",  "int16",  "inverter"),
    _r(0x0120, "pv1_voltage",    "PV1 Voltage",               0.1,  "V",   "uint16", "pv"),
    _r(0x0121, "pv1_current",    "PV1 Current",               0.01, "A",   "uint16", "pv"),
    _r(0x0122, "pv1_power",      "PV1 Power",                 1.0,  "W",   "int32",  "pv"),
    _r(0x0124, "pv2_voltage",    "PV2 Voltage",               0.1,  "V",   "uint16", "pv"),
    _r(0x0125, "pv2_current",    "PV2 Current",               0.01, "A",   "uint16", "pv"),
    _r(0x0126, "pv2_power",      "PV2 Power",                 1.0,  "W",   "int32",  "pv"),
    _r(0x0140, "bat_voltage",    "Battery Voltage",           0.01, "V",   "uint16", "battery"),
    _r(0x0141, "bat_current",    "Battery Current",           0.01, "A",   "int16",  "battery"),
    _r(0x0142, "bat_power",      "Battery Power",             1.0,  "W",   "int32",  "battery"),
    _r(0x0144, "bat_temp",       "Battery Temperature",       0.1,  "°C",  "int16",  "battery"),
    _r(0x0148, "soc",            "Battery SOC",               0.01, "%",   "uint16", "battery"),
    _r(0x0150, "bdc_status",     "Battery Direction",         1.0,  "",    "uint16", "battery", "bdc_status"),
]

_SAJ_LOOKUPS = {
    "run_mode": {
        0: "Standby", 1: "Normal", 2: "Fault", 3: "Upgrade", 4: "Self-test",
    },
    "bdc_status": {0: "Charging", 1: "Discharging", 2: "Standby"},
}

_SAJ_DEMO: dict[int, int] = {
    0x0100: 1,    0x0101: 2310, 0x0102: 2308, 0x0103: 2312,
    0x0104: 148,  0x0105: 146,  0x0106: 150,
    0x0107: 3400, 0x0108: 0,    0x0109: 200,  0x010A: 0,
    0x010B: 998,  0x010C: 5001, 0x0115: 410,
    0x0120: 3850, 0x0121: 88,   0x0122: 3381, 0x0123: 0,
    0x0124: 3720, 0x0125: 62,   0x0126: 2306, 0x0127: 0,
    0x0140: 5200, 0x0141: 80,   0x0142: 416,  0x0143: 0,
    0x0144: 280,  0x0148: 7200, 0x0150: 1,
}

_SAJ_SNAP_MAP = {
    "grid_voltage":   (0x0101, "uint16", 0.1),
    "grid_current":   (0x0104, "int16",  0.01),
    "grid_power":     (0x0107, "int32",  1.0),
    "grid_frequency": (0x010C, "uint16", 0.01),
    "power_factor":   (0x010B, "int16",  0.001),
    "inv_temp":       (0x0115, "int16",  0.1),
    "pv1_voltage":    (0x0120, "uint16", 0.1),
    "pv1_current":    (0x0121, "uint16", 0.01),
    "pv1_power":      (0x0122, "int32",  1.0),
    "pv2_voltage":    (0x0124, "uint16", 0.1),
    "pv2_current":    (0x0125, "uint16", 0.01),
    "pv2_power":      (0x0126, "int32",  1.0),
    "bat_voltage":    (0x0140, "uint16", 0.01),
    "bat_current":    (0x0141, "int16",  0.01),
    "bat_power":      (0x0142, "int32",  1.0),
    "bat_temp":       (0x0144, "int16",  0.1),
    "soc":            (0x0148, "uint16", 0.01),
}
_SAJ_STATUS = {"run_mode": 0x0100, "bdc_status": 0x0150}

SAJ = InverterProfile(
    name          = "SAJ H1/R5/HS2",
    input_regs    = {r.addr: r for r in _SAJ_INPUT},
    input_chunks  = [
        (0x0100, 0x0020), (0x0140, 0x0015),
    ],
    lookups       = _SAJ_LOOKUPS,
    demo_words    = _SAJ_DEMO,
    build_snapshot= lambda w: _build_generic_snapshot(w, _SAJ_SNAP_MAP, _SAJ_STATUS),
)


# ═══════════════════════════════════════════════════════════════════════════
# FOX ESS H1/AC1/T-Series  —  Modbus RTU
# Ref: FoxESS Modbus Protocol V1.0 (public)
# ═══════════════════════════════════════════════════════════════════════════

_FOX_INPUT = [
    _r(0x0000, "pv1_voltage",    "PV1 Voltage",               0.1,  "V",   "uint16", "pv"),
    _r(0x0001, "pv1_current",    "PV1 Current",               0.1,  "A",   "int16",  "pv"),
    _r(0x0002, "pv1_power",      "PV1 Power",                 1.0,  "W",   "int16",  "pv"),
    _r(0x0003, "pv2_voltage",    "PV2 Voltage",               0.1,  "V",   "uint16", "pv"),
    _r(0x0004, "pv2_current",    "PV2 Current",               0.1,  "A",   "int16",  "pv"),
    _r(0x0005, "pv2_power",      "PV2 Power",                 1.0,  "W",   "int16",  "pv"),
    _r(0x0006, "grid_voltage_r", "Grid Voltage R",            0.1,  "V",   "uint16", "grid"),
    _r(0x0007, "grid_voltage_s", "Grid Voltage S",            0.1,  "V",   "uint16", "grid"),
    _r(0x0008, "grid_voltage_t", "Grid Voltage T",            0.1,  "V",   "uint16", "grid"),
    _r(0x0009, "grid_current_r", "Grid Current R",            0.1,  "A",   "int16",  "grid"),
    _r(0x000A, "grid_current_s", "Grid Current S",            0.1,  "A",   "int16",  "grid"),
    _r(0x000B, "grid_current_t", "Grid Current T",            0.1,  "A",   "int16",  "grid"),
    _r(0x000C, "grid_power",     "Active Power",              1.0,  "W",   "int16",  "grid"),
    _r(0x000D, "reactive_power", "Reactive Power",            1.0,  "var", "int16",  "grid"),
    _r(0x000E, "grid_frequency", "Grid Frequency",            0.01, "Hz",  "uint16", "grid"),
    _r(0x000F, "power_factor",   "Power Factor",              0.001,"",    "int16",  "grid"),
    _r(0x0010, "inv_temp",       "Inverter Temp",             0.1,  "°C",  "int16",  "inverter"),
    _r(0x0011, "run_mode",       "Inverter State",            1.0,  "",    "uint16", "inverter", "run_mode"),
    _r(0x001A, "bat_voltage",    "Battery Voltage",           0.1,  "V",   "uint16", "battery"),
    _r(0x001B, "bat_current",    "Battery Current",           0.1,  "A",   "int16",  "battery"),
    _r(0x001C, "bat_power",      "Battery Power",             1.0,  "W",   "int16",  "battery"),
    _r(0x001D, "bat_temp",       "Battery Temperature",       0.1,  "°C",  "int16",  "battery"),
    _r(0x001E, "soc",            "Battery SOC",               1.0,  "%",   "uint16", "battery"),
    _r(0x001F, "bdc_status",     "Battery State",             1.0,  "",    "uint16", "battery", "bdc_status"),
]

_FOX_LOOKUPS = {
    "run_mode": {
        0: "Standby", 1: "Normal", 2: "Fault", 3: "Flash",
        4: "Check",   5: "Off",
    },
    "bdc_status": {0: "Standby", 1: "Charging", 2: "Discharging", 3: "Fault"},
}

_FOX_DEMO: dict[int, int] = {
    0x0000: 3850, 0x0001: 85,   0x0002: 3270,
    0x0003: 3720, 0x0004: 60,   0x0005: 2232,
    0x0006: 2310, 0x0007: 2308, 0x0008: 2312,
    0x0009: 148,  0x000A: 146,  0x000B: 150,
    0x000C: 3400, 0x000D: 200,  0x000E: 5001,
    0x000F: 998,  0x0010: 420,  0x0011: 1,
    0x001A: 520,  0x001B: 80,   0x001C: 416,
    0x001D: 280,  0x001E: 72,   0x001F: 1,
}

_FOX_SNAP_MAP = {
    "grid_voltage":   (0x0006, "uint16", 0.1),
    "grid_current":   (0x0009, "int16",  0.1),
    "grid_power":     (0x000C, "int16",  1.0),
    "grid_frequency": (0x000E, "uint16", 0.01),
    "power_factor":   (0x000F, "int16",  0.001),
    "inv_temp":       (0x0010, "int16",  0.1),
    "pv1_voltage":    (0x0000, "uint16", 0.1),
    "pv1_current":    (0x0001, "int16",  0.1),
    "pv1_power":      (0x0002, "int16",  1.0),
    "pv2_voltage":    (0x0003, "uint16", 0.1),
    "pv2_current":    (0x0004, "int16",  0.1),
    "pv2_power":      (0x0005, "int16",  1.0),
    "bat_voltage":    (0x001A, "uint16", 0.1),
    "bat_current":    (0x001B, "int16",  0.1),
    "bat_power":      (0x001C, "int16",  1.0),
    "bat_temp":       (0x001D, "int16",  0.1),
    "soc":            (0x001E, "uint16", 1.0),
}
_FOX_STATUS = {"run_mode": 0x0011, "bdc_status": 0x001F}

FOX_ESS = InverterProfile(
    name          = "Fox ESS H1/AC1/T",
    input_regs    = {r.addr: r for r in _FOX_INPUT},
    input_chunks  = [(0x0000, 0x0020)],
    lookups       = _FOX_LOOKUPS,
    demo_words    = _FOX_DEMO,
    build_snapshot= lambda w: _build_generic_snapshot(w, _FOX_SNAP_MAP, _FOX_STATUS),
)


# ═══════════════════════════════════════════════════════════════════════════
# PROFILE REGISTRY  —  add new brands here
# ═══════════════════════════════════════════════════════════════════════════

PROFILES: dict[str, Optional[InverterProfile]] = {
    "Generic (raw Modbus)": None,
    "SolaX X1 Hybrid G4":  SOLAX_X1,
    "Huawei SUN2000":       HUAWEI_SUN2000,
    "Deye SUN Series":      DEYE_SUN,
    "Solis S5/S6":          SOLIS,
    "SAJ H1/R5/HS2":        SAJ,
    "Fox ESS H1/AC1/T":     FOX_ESS,
}


# ═══════════════════════════════════════════════════════════════════════════
# UI  —  page config + CSS
# ═══════════════════════════════════════════════════════════════════════════

st.set_page_config(
    page_title="Modbus RTU Inspector",
    page_icon="",
    layout="wide",
    initial_sidebar_state="expanded",
)

st.markdown("""
<style>
  [data-testid="metric-container"] {
      background: #1c1c1c; border: 1px solid #2e2e2e;
      border-radius: 6px; padding: 12px 16px;
  }
  [data-testid="stMetricLabel"] { font-size: 0.72rem; color: #888;
      text-transform: uppercase; letter-spacing: 0.05em; }
  [data-testid="stMetricValue"] { font-size: 1.3rem; font-weight: 600; color: #e0e0e0; }
  [data-testid="stMetricDelta"] { font-size: 0.72rem; }
  [data-testid="stTabs"] button[role="tab"] { font-weight: 500; font-size: 0.85rem; }
  [data-testid="stSidebar"] { background: #141414; }

  .section-header {
      font-size: 0.7rem; font-weight: 600; letter-spacing: 0.15em;
      text-transform: uppercase; color: #555;
      margin: 20px 0 10px 0; padding-bottom: 5px;
      border-bottom: 1px solid #2a2a2a;
  }
  .badge-ok  { background: transparent; color: #aaa; border: 1px solid #3a3a3a;
               border-radius: 4px; padding: 2px 10px; font-size: 0.78rem; }
  .badge-err { background: transparent; color: #b85c5c; border: 1px solid #b85c5c;
               border-radius: 4px; padding: 2px 10px; font-size: 0.78rem; }
  .soc-wrap  { background: #222; border-radius: 4px; height: 18px;
               margin: 4px 0 16px 0; overflow: hidden; }
  .soc-fill  { height: 100%; border-radius: 4px; display: flex;
               align-items: center; justify-content: center;
               color: #ccc; font-size: 11px; font-weight: 600;
               transition: width 0.5s ease; }
</style>
""", unsafe_allow_html=True)


# ── UI helpers ─────────────────────────────────────────────────────────────

def _section(label: str):
    st.markdown(f'<div class="section-header">{label}</div>', unsafe_allow_html=True)


def _badge(text: str, level: str = "ok"):
    cls = "badge-ok" if level == "ok" else "badge-err"
    st.markdown(f'<span class="{cls}">{text}</span>', unsafe_allow_html=True)


def _soc_bar(pct: float):
    pct = max(0, min(100, int(pct)))
    color = "#555" if pct > 50 else ("#4a4a4a" if pct > 20 else "#7a3a3a")
    st.markdown(
        f'<div class="soc-wrap"><div class="soc-fill" style="width:{pct}%;background:{color};">'
        f'{pct}%</div></div>',
        unsafe_allow_html=True,
    )


def _fmt(value, unit: str = "") -> str:
    if value is None:
        return "—"
    return f"{value} {unit}".strip()


# ═══════════════════════════════════════════════════════════════════════════
# SIDEBAR
# ═══════════════════════════════════════════════════════════════════════════

with st.sidebar:
    st.markdown("### Modbus RTU Inspector")
    st.divider()

    profile_name = st.selectbox("Inverter Profile", list(PROFILES.keys()))
    profile: Optional[InverterProfile] = PROFILES[profile_name]

    st.caption(f"Profile: {profile.name}" if profile else "No profile — raw mode only.")
    st.divider()

    mode = st.radio("Connection", ["Live Serial", "Demo / Offline"],
                    help="Demo uses the profile's built-in sample data.")
    live_mode = (mode == "Live Serial")

    if live_mode and not SERIAL_AVAILABLE:
        st.error("pyserial not installed.\npip install pyserial")
        live_mode = False

    if live_mode:
        ports        = [p.device for p in serial.tools.list_ports.comports()] or ["COM4"]
        default_port = "COM4" if "COM4" in ports else ports[0]
        port         = st.selectbox("Port",  ports, index=ports.index(default_port))
        baud         = st.selectbox("Baud",  list(VALID_BAUDS), index=list(VALID_BAUDS).index(9600))
        slave_id     = st.number_input("Slave ID", 1, 247, 1)
        timeout      = st.slider("Timeout (s)", 0.5, 5.0, 1.0, 0.5)
        retries      = st.number_input("Retries", 0, 5, 2)
    else:
        port, baud, slave_id, timeout, retries = "COM4", 9600, 1, 1.0, 2

    st.divider()
    st.caption("Min poll interval: 1 s\nEEprom registers have limited write cycles.")


# ── Session state ──────────────────────────────────────────────────────────
for _k in ("last_response", "last_frame_sent", "last_start_reg", "last_fc"):
    if _k not in st.session_state:
        st.session_state[_k] = None

if "history" not in st.session_state:
    st.session_state.history = {
        k: deque(maxlen=60) for k in ("ts", "pv_power", "bat_power", "grid_power", "soc")
    }


# ═══════════════════════════════════════════════════════════════════════════
# TABS
# ═══════════════════════════════════════════════════════════════════════════

tab1, tab2, tab3, tab4 = st.tabs([
    "Frame Builder",
    "Response Decoder",
    "Dashboard",
    "Bulk Scan",
])


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  TAB 1 — FRAME BUILDER                                                 ║
# ╚══════════════════════════════════════════════════════════════════════════╝
with tab1:
    st.markdown("#### Frame Builder")
    st.caption("Build and send any Modbus RTU frame. The last response is forwarded to the Response Decoder tab.")

    if not live_mode:
        st.info("Offline mode — frame preview is available but sending is disabled.")

    with st.expander("Frame structure reference"):
        st.markdown("""
| Field | Bytes | Notes |
|---|---|---|
| Slave ID | 1 | Default `0x01` |
| Function Code | 1 | `0x03` Holding / `0x04` Input / `0x06` Write Single |
| Register (MSB) | 1 | High byte of start address |
| Register (LSB) | 1 | Low byte |
| Count / Value (MSB) | 1 | Number of registers to read, or write value high byte |
| Count / Value (LSB) | 1 | |
| CRC-16 Low | 1 | Auto-computed |
| CRC-16 High | 1 | |
""")

    c1, c2 = st.columns(2)
    with c1:
        fb_fc  = st.selectbox("Function Code", [
            "0x03 — Read Holding",
            "0x04 — Read Input",
            "0x06 — Write Single Register",
        ])
        fb_reg = st.text_input("Register (hex)", value="0000", placeholder="e.g. 0046")
        fb_val = st.number_input("Count (read) / Value (write)", 0, 65535, 1)
    with c2:
        fb_rpt      = st.number_input("Repeat", 1, 50, 1)
        fb_delay    = st.number_input("Delay between repeats (ms)", 100, 5000, 500)
        fb_raw_only = st.checkbox("Raw hex only — skip auto-decode")

    frame_preview: Optional[bytes] = None
    try:
        fc_b          = int(fb_fc[:4], 16)
        reg_b         = int(fb_reg.strip(), 16)
        frame_preview = build_frame(int(slave_id), fc_b, reg_b, int(fb_val))
        st.code(f"{frame_preview.hex(' ').upper()}  ({len(frame_preview)} bytes)")
    except Exception as exc:
        st.warning(f"Frame build error: {exc}")

    if st.button("Send", disabled=(frame_preview is None or not live_mode), use_container_width=True):
        log_rows, last_resp = [], None
        prog = st.progress(0)

        for i in range(int(fb_rpt)):
            frame, resp = send_frame(
                port, int(baud), int(slave_id), fc_b, reg_b, int(fb_val),
                timeout=float(timeout), retries=0, gap=0.12,
            )
            if resp:
                ok = crc_valid(resp, fc_b)
                last_resp = resp
                log_rows.append({"#": i + 1, "Sent": frame.hex(" ").upper(),
                                  "Response": resp.hex(" ").upper(),
                                  "CRC": "OK" if ok else "FAIL", "Bytes": len(resp)})
            else:
                log_rows.append({"#": i + 1, "Sent": frame.hex(" ").upper(),
                                  "Response": "NO RESPONSE", "CRC": "—", "Bytes": 0})
            prog.progress((i + 1) / int(fb_rpt))
            if i < int(fb_rpt) - 1:
                time.sleep(int(fb_delay) / 1000)

        st.dataframe(pd.DataFrame(log_rows), use_container_width=True, hide_index=True)

        if last_resp and not fb_raw_only:
            st.session_state.last_response   = last_resp.hex(" ").upper()
            st.session_state.last_frame_sent = frame_preview.hex(" ").upper()
            st.session_state.last_start_reg  = reg_b
            st.session_state.last_fc         = fc_b
            st.success("Response forwarded to Response Decoder tab.")


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  TAB 2 — RESPONSE DECODER                                              ║
# ╚══════════════════════════════════════════════════════════════════════════╝
with tab2:
    st.markdown("#### Response Decoder")
    st.caption(
        "Paste a raw hex response or use the one forwarded from the Frame Builder. "
        "Register names and scaled values are shown when a profile is selected."
    )

    dec_source = st.radio("Source", ["Last Frame Builder response", "Paste hex"], horizontal=True)

    hex_input = ""
    start_reg: Optional[int] = None
    fc_hint   = FC_READ_INPUT

    if dec_source == "Last Frame Builder response":
        if st.session_state.last_response:
            hex_input = st.session_state.last_response
            start_reg = st.session_state.last_start_reg
            fc_hint   = st.session_state.last_fc or FC_READ_INPUT
            st.code(hex_input)
            st.caption(f"Start reg: 0x{start_reg:04X}   FC: 0x{fc_hint:02X}")
        else:
            st.info("No response yet. Send a frame in the Frame Builder tab first.")

    else:  # Paste hex
        with st.expander("Example responses"):
            st.code("Input regs:   01 04 06 00 00 00 00 00 00 60 93")
            st.code("Holding regs: 01 03 0E 48 34 37 35 32 32 5A 48 45 4E 47 57 45 4E 63 26")
            st.code("Write single: 01 06 00 1F 00 00 48 0A")

        hex_input = st.text_area("Hex bytes (spaces or colons optional)",
                                 placeholder="01 04 14 00 EB ...", height=80)
        rc1, rc2 = st.columns(2)
        with rc1:
            start_hex = st.text_input("Start register (hex)", placeholder="e.g. 0000")
            try:
                start_reg = int(start_hex.strip(), 16) if start_hex.strip() else None
            except ValueError:
                start_reg = None
        with rc2:
            fc_label = st.selectbox("Function code", ["0x04 Input", "0x03 Holding", "0x06 Write"])
            fc_hint  = int(fc_label[:4], 16)

    if hex_input.strip():
        if st.button("Decode", use_container_width=True):
            clean = hex_input.replace(" ", "").replace(":", "").replace("-", "")
            try:
                raw_b = bytes.fromhex(clean)
            except ValueError:
                st.error("Invalid hex.")
                st.stop()
            if len(raw_b) < 4:
                st.error("Frame too short.")
                st.stop()

            slave_r = raw_b[0]
            fc_r    = raw_b[1]
            is_ok   = crc_valid(raw_b, fc_r)

            _section("Frame Summary")
            m1, m2, m3, m4 = st.columns(4)
            m1.metric("Slave ID",      f"0x{slave_r:02X} ({slave_r})")
            m2.metric("Function Code", f"0x{fc_r:02X}")
            m3.metric("Total Bytes",   len(raw_b))
            m4.metric("CRC",           "Valid" if is_ok else "Invalid")

            if fc_r in (0x83, 0x84):
                st.error(f"Exception response — error code: 0x{raw_b[2]:02X}")

            elif fc_r in (FC_READ_INPUT, FC_READ_HOLDING):
                words   = parse_words(raw_b)
                ref_map = (profile.input_regs if fc_r == FC_READ_INPUT else profile.holding_regs) \
                          if profile else {}

                rows = []
                for idx, w in enumerate(words):
                    addr = (start_reg + idx) if start_reg is not None else idx
                    row: dict = {
                        "Offset": idx,
                        "Address": f"0x{addr:04X}" if start_reg is not None else "—",
                        "uint16": w, "int16": as_int16(w),
                        "Hex": f"0x{w:04X}",
                        "x0.1": round(w * 0.1, 1), "x0.01": round(w * 0.01, 2),
                    }
                    if start_reg is not None and addr in ref_map and profile:
                        reg    = ref_map[addr]
                        scaled = decode_reg({addr: w}, reg)
                        label  = resolve_label(profile, reg, w)
                        row.update({
                            "Name": reg.name, "Category": reg.category,
                            "Scaled": _fmt(scaled, reg.unit),
                            "Value": label or _fmt(scaled, reg.unit),
                            "Description": reg.desc,
                        })
                    rows.append(row)

                st.markdown(f"{len(words)} register(s)")
                st.dataframe(pd.DataFrame(rows), use_container_width=True, hide_index=True)

                try:
                    chars     = b"".join(bytes([(w >> 8) & 0xFF, w & 0xFF]) for w in words)
                    printable = "".join(c if 32 <= ord(c) < 127 else "." for c in chars.decode("latin-1"))
                    with st.expander("ASCII"):
                        st.code(printable)
                except Exception:
                    pass

            elif fc_r == 0x06 and len(raw_b) >= 6:
                reg_a = (raw_b[2] << 8) | raw_b[3]
                val_w = (raw_b[4] << 8) | raw_b[5]
                _section("Write Acknowledgement")
                wa, wb = st.columns(2)
                wa.metric("Register", f"0x{reg_a:04X} ({reg_a})")
                wb.metric("Value",    f"0x{val_w:04X} ({val_w})")


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  TAB 3 — DASHBOARD                                                     ║
# ╚══════════════════════════════════════════════════════════════════════════╝
with tab3:
    st.markdown("#### Dashboard")

    if profile is None:
        st.info("Select an inverter profile in the sidebar to enable the dashboard.")
        st.stop()

    dc1, dc2, dc3 = st.columns([2, 1, 1])
    with dc1:
        poll_interval = st.slider("Refresh (s)", 3, 120, 10, label_visibility="collapsed")
        st.caption(f"Refresh every {poll_interval} s")
    with dc2:
        auto_refresh = st.toggle("Auto-refresh", value=False)
    with dc3:
        do_poll = st.button("Poll now", use_container_width=True)

    if auto_refresh:
        time.sleep(poll_interval)
        st.rerun()

    snap: Optional[dict] = None

    if not live_mode:
        if profile.demo_words:
            snap = profile.build_snapshot(profile.demo_words)
            st.caption("Demo data — switch to Live Serial to poll hardware.")
        else:
            st.warning("No demo data for this profile.")
    elif do_poll or auto_refresh:
        with st.spinner(f"Polling {port} @ {baud}..."):
            words = poll_chunks(port, int(baud), int(slave_id),
                                FC_READ_INPUT, profile.input_chunks,
                                float(timeout), int(retries), 0.12)
        if words:
            snap = profile.build_snapshot(words)
        else:
            st.error("No response from inverter. Check cable, port, and baud rate.")

    if snap is None:
        st.info("Press 'Poll now' or enable Auto-refresh to load data.")
        st.stop()

    h  = st.session_state.history
    lk = profile.lookups

    h["ts"].append(snap["timestamp"].strftime("%H:%M:%S"))
    h["pv_power"].append(snap.get("pv_total", 0))
    h["bat_power"].append(snap.get("bat_power") or 0)
    h["grid_power"].append(snap.get("grid_power") or 0)
    h["soc"].append(snap.get("soc") or 0)

    # Status strip
    _section("Status")
    s1, s2, s3, s4, s5 = st.columns(5)
    s1.metric("Run Mode",  lk.get("run_mode",   {}).get(snap.get("run_mode"),   str(snap.get("run_mode", "—"))))
    s2.metric("Grid",      lk.get("grid_status",{}).get(snap.get("grid_status"),"—"))
    s3.metric("Battery",   lk.get("bdc_status", {}).get(snap.get("bdc_status"), "—"))
    s4.metric("Inv Temp",  _fmt(snap.get("inv_temp"), "°C"))
    s5.metric("Last Poll", snap["timestamp"].strftime("%H:%M:%S"))

    st.divider()

    # Grid
    _section("Grid")
    g1, g2, g3, g4, g5 = st.columns(5)
    g1.metric("Voltage",      _fmt(snap.get("grid_voltage"),   "V"))
    g2.metric("Current",      _fmt(snap.get("grid_current"),   "A"))
    g3.metric("Power",        _fmt(snap.get("grid_power"),     "W"))
    g4.metric("Frequency",    _fmt(snap.get("grid_frequency"), "Hz"))
    pf = snap.get("power_factor")
    g5.metric("Power Factor", _fmt(pf))

    fi = snap.get("feedin_power")
    if fi is not None:
        st.caption(f"Feed-in: {abs(fi)} W  ({'Export' if fi >= 0 else 'Import'})")

    st.divider()

    # PV
    _section("Solar PV")
    p1, p2, p3 = st.columns(3)
    with p1:
        st.caption("String 1")
        a, b, c = st.columns(3)
        a.metric("Voltage", _fmt(snap.get("pv1_voltage"), "V"))
        b.metric("Current", _fmt(snap.get("pv1_current"), "A"))
        c.metric("Power",   _fmt(snap.get("pv1_power"),   "W"))
    with p2:
        st.caption("String 2")
        a, b, c = st.columns(3)
        a.metric("Voltage", _fmt(snap.get("pv2_voltage"), "V"))
        b.metric("Current", _fmt(snap.get("pv2_current"), "A"))
        c.metric("Power",   _fmt(snap.get("pv2_power"),   "W"))
    with p3:
        st.caption("Total")
        st.metric("PV Total", _fmt(snap.get("pv_total"), "W"))

    st.divider()

    # Battery
    _section("Battery")
    b1, b2, b3, b4, b5 = st.columns(5)
    b1.metric("Voltage",     _fmt(snap.get("bat_voltage"), "V"))
    b2.metric("Current",     _fmt(snap.get("bat_current"), "A"))
    b3.metric("Power",       _fmt(snap.get("bat_power"),   "W"))
    b4.metric("Temperature", _fmt(snap.get("bat_temp"),   "°C"))
    b5.metric("SOC",         _fmt(snap.get("soc"),         "%"))
    _soc_bar(snap.get("soc") or 0)

    st.divider()

    # Trend
    if len(h["ts"]) > 1:
        _section("Trend (last 60 polls)")
        df_trend = pd.DataFrame({
            "Time":        list(h["ts"]),
            "PV (W)":      list(h["pv_power"]),
            "Battery (W)": list(h["bat_power"]),
            "Grid (W)":    list(h["grid_power"]),
            "SOC (%)":     list(h["soc"]),
        }).set_index("Time")
        ch1, ch2 = st.columns(2)
        ch1.line_chart(df_trend[["PV (W)", "Battery (W)", "Grid (W)"]], height=200)
        ch2.line_chart(df_trend[["SOC (%)"]], height=200)

    # Raw dump
    if snap.get("_words"):
        with st.expander("Raw register dump"):
            raw_df = pd.DataFrame([
                {"Addr": f"0x{k:04X}", "Dec": k,
                 "uint16": v, "int16": as_int16(v), "Hex": f"0x{v:04X}"}
                for k, v in sorted(snap["_words"].items())
            ])
            st.dataframe(raw_df, use_container_width=True, hide_index=True)


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  TAB 4 — BULK SCAN                                                     ║
# ╚══════════════════════════════════════════════════════════════════════════╝
with tab4:
    st.markdown("#### Bulk Register Scan")
    st.caption("Scan a register range in chunks. Useful for discovering unknown registers.")

    if not live_mode:
        st.warning("Bulk scan requires Live Serial mode.")
        st.stop()

    sc1, sc2, sc3 = st.columns(3)
    with sc1:
        scan_fc_label = st.selectbox("Function Code", ["0x04 — Read Input", "0x03 — Read Holding"])
        scan_fc = int(scan_fc_label[:4], 16)
    with sc2:
        scan_start = st.text_input("Start (hex)", value="0000")
        scan_end   = st.text_input("End (hex)",   value="00FF")
    with sc3:
        scan_chunk = st.number_input("Chunk size", 1, 50, 20)
        st.markdown("<br>", unsafe_allow_html=True)
        do_scan = st.button("Start Scan", use_container_width=True)

    if do_scan:
        try:
            start_a = int(scan_start.strip(), 16)
            end_a   = int(scan_end.strip(),   16)
        except ValueError:
            st.error("Invalid address.")
            st.stop()
        if end_a <= start_a:
            st.error("End must be greater than start.")
            st.stop()

        chunks = [
            (addr, min(int(scan_chunk), end_a - addr + 1))
            for addr in range(start_a, end_a + 1, int(scan_chunk))
        ]
        ref_map = (profile.input_regs if scan_fc == FC_READ_INPUT else profile.holding_regs) \
                   if profile else {}

        all_words: dict[int, int] = {}
        prog = st.progress(0)

        for i, (chunk_start, chunk_count) in enumerate(chunks):
            _, resp = send_frame(
                port, int(baud), int(slave_id), scan_fc, chunk_start, chunk_count,
                timeout=float(timeout), retries=int(retries), gap=0.12,
            )
            if resp:
                for j, w in enumerate(parse_words(resp)):
                    all_words[chunk_start + j] = w
            prog.progress((i + 1) / len(chunks))
            time.sleep(0.12)

        if not all_words:
            st.error("No data received.")
            st.stop()

        rows = []
        for addr, w in sorted(all_words.items()):
            row: dict = {
                "Address": f"0x{addr:04X}", "uint16": w,
                "int16": as_int16(w), "Hex": f"0x{w:04X}",
                "x0.1": round(w * 0.1, 1), "x0.01": round(w * 0.01, 2),
            }
            if addr in ref_map and profile:
                reg    = ref_map[addr]
                scaled = decode_reg({addr: w}, reg)
                label  = resolve_label(profile, reg, w)
                row.update({
                    "Name": reg.name, "Category": reg.category,
                    "Scaled": _fmt(scaled, reg.unit),
                    "Value": label or _fmt(scaled, reg.unit),
                    "Description": reg.desc,
                })
            rows.append(row)

        st.markdown(f"{len(rows)} registers  (0x{start_a:04X} – 0x{end_a:04X}, FC 0x{scan_fc:02X})")
        st.dataframe(pd.DataFrame(rows), use_container_width=True, hide_index=True)


# ── Footer ─────────────────────────────────────────────────────────────────
st.divider()
st.caption("Modbus RTU Inspector  |  Min frame interval: 1 s  |  EEprom registers have limited write cycles")
