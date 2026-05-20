"""
Modbus RTU Inspector
====================
Generic Modbus RTU frame builder, sender, and response decoder.
Inverter-specific register maps are loaded as optional profiles —
select one in the sidebar to enable register decoding and dashboards.

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
    """
    Send one Modbus RTU request, return (frame_sent, response | None).
    Retries on empty or bad-CRC responses.
    """
    frame = build_frame(slave, fc, register, value)
    expected = 5 + value * 2 if fc != FC_WRITE_SINGLE else 8

    for attempt in range(1 + retries):
        try:
            with serial.Serial(port, baud, timeout=timeout) as conn:
                time.sleep(gap)
                conn.write(frame)
                resp = conn.read(expected)
            if resp and crc_valid(resp, fc):
                return frame, resp
            log.debug(
                "attempt %d: bad/empty response for reg 0x%04X (%d bytes)",
                attempt + 1, register, len(resp) if resp else 0,
            )
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
    """
    Self-contained inverter definition.
    Plug in a new profile to add support for any inverter.
    """
    name:           str
    input_regs:     dict[int, RegDef]         = field(default_factory=dict)
    holding_regs:   dict[int, RegDef]         = field(default_factory=dict)
    input_chunks:   list[tuple[int, int]]     = field(default_factory=list)
    holding_chunks: list[tuple[int, int]]     = field(default_factory=list)
    lookups:        dict[str, dict[int, str]] = field(default_factory=dict)
    fault_bit_maps: dict[str, dict[int, str]] = field(default_factory=dict)
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


def active_faults(word32: int, bit_map: dict[int, str]) -> list[str]:
    return [bit_map[b] for b in sorted(bit_map) if (word32 >> b) & 1]


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
# SOLAX X1 HYBRID G4  —  Protocol V3.21, single-phase
# ═══════════════════════════════════════════════════════════════════════════

def _r(addr, name, desc, scale, unit, dtype, cat, lookup=None):
    return RegDef(addr, name, desc, scale, unit, dtype, cat, lookup)


_SOLAX_INPUT = [
    _r(0x0000, "grid_voltage",    "Grid RMS Voltage",            0.1,   "V",   "uint16", "grid"),
    _r(0x0001, "grid_current",    "Grid RMS Current",            0.1,   "A",   "int16",  "grid"),
    _r(0x0002, "grid_power",      "Grid Active Power",           1.0,   "W",   "int16",  "grid"),
    _r(0x0007, "grid_frequency",  "Grid Frequency",              0.01,  "Hz",  "uint16", "grid"),
    _r(0x001A, "grid_status",     "On-Grid / Off-Grid",          1.0,   "",    "uint16", "grid",    "grid_status"),
    _r(0x0046, "feedin_power",    "Feed-in Power (meter/CT)",    1.0,   "W",   "int32",  "grid"),
    _r(0x0003, "pv1_voltage",     "PV String 1 Voltage",         0.1,   "V",   "uint16", "pv"),
    _r(0x0004, "pv2_voltage",     "PV String 2 Voltage",         0.1,   "V",   "uint16", "pv"),
    _r(0x0005, "pv1_current",     "PV String 1 Current",         0.1,   "A",   "uint16", "pv"),
    _r(0x0006, "pv2_current",     "PV String 2 Current",         0.1,   "A",   "uint16", "pv"),
    _r(0x000A, "pv1_power",       "DC Power String 1",           1.0,   "W",   "uint16", "pv"),
    _r(0x000B, "pv2_power",       "DC Power String 2",           1.0,   "W",   "uint16", "pv"),
    _r(0x0008, "inv_temp",        "Radiator Temperature",        1.0,   "°C",  "int16",  "inverter"),
    _r(0x0009, "run_mode",        "Inverter Run Mode",           1.0,   "",    "uint16", "inverter", "run_mode"),
    _r(0x001B, "mppt_count",      "Number of MPPT Channels",     1.0,   "",    "uint16", "inverter"),
    _r(0x0014, "bat_voltage",     "Battery Voltage",             0.1,   "V",   "int16",  "battery"),
    _r(0x0015, "bat_current",     "Battery Current (+chg/-dis)", 0.1,   "A",   "int16",  "battery"),
    _r(0x0016, "bat_power",       "Battery Power (+chg/-dis)",   1.0,   "W",   "int16",  "battery"),
    _r(0x0018, "bat_temp",        "Battery Temperature",         1.0,   "°C",  "int16",  "battery"),
    _r(0x001C, "soc",             "State of Charge",             1.0,   "%",   "uint16", "battery"),
    _r(0x0019, "bdc_status",      "Battery Direction",           1.0,   "",    "uint16", "battery",  "bdc_status"),
    _r(0x0017, "bms_connected",   "BMS Connection State",        1.0,   "",    "uint16", "battery",  "bms_connected"),
    _r(0x0024, "bms_max_chg_i",   "BMS Charge Max Current",      0.1,   "A",   "uint16", "battery"),
    _r(0x0025, "bms_max_dis_i",   "BMS Discharge Max Current",   0.1,   "A",   "uint16", "battery"),
    _r(0x00BA, "bat_temp_high",   "Battery High Temperature",    0.1,   "°C",  "int16",  "battery"),
    _r(0x00BB, "bat_temp_low",    "Battery Low Temperature",     0.1,   "°C",  "int16",  "battery"),
    _r(0x00BC, "cell_v_high",     "Highest Cell Voltage",        0.001, "V",   "uint16", "battery"),
    _r(0x00BD, "cell_v_low",      "Lowest Cell Voltage",         0.001, "V",   "uint16", "battery"),
    _r(0x00BE, "bms_soc",         "BMS User SOC",                1.0,   "%",   "uint16", "battery"),
    _r(0x00BF, "bms_soh",         "BMS User SOH",                1.0,   "%",   "uint16", "battery"),
    _r(0x004C, "offgrid_voltage", "Off-grid Output Voltage",     0.1,   "V",   "uint16", "offgrid"),
    _r(0x004D, "offgrid_current", "Off-grid Output Current",     0.1,   "A",   "uint16", "offgrid"),
    _r(0x004E, "offgrid_power",   "Off-grid Output Power",       1.0,   "VA",  "uint16", "offgrid"),
    _r(0x004F, "offgrid_freq",    "Off-grid Output Frequency",   0.01,  "Hz",  "uint16", "offgrid"),
    _r(0x0050, "yield_today",     "Today Energy - AC Port",      0.1,   "kWh", "uint16", "energy"),
    _r(0x0096, "solar_today",     "Solar Energy Today",          0.1,   "kWh", "uint16", "energy"),
    _r(0x0020, "bat_chg_today",   "Battery Charge Today",        0.1,   "kWh", "uint16", "energy"),
    _r(0x0023, "bat_dis_today",   "Battery Discharge Today",     0.1,   "kWh", "uint16", "energy"),
    _r(0x0091, "echarge_today",   "Charge Energy Today (AC)",    0.1,   "kWh", "uint16", "energy"),
    _r(0x0090, "offgrid_today",   "Off-grid Yield Today",        0.1,   "kWh", "uint16", "energy"),
    _r(0x003E, "pcs_fault",       "PCS Major Fault Code",        1.0,   "",    "uint16", "faults"),
    _r(0x003F, "bat_fault",       "Battery Major Fault Code",    1.0,   "",    "uint16", "faults"),
    _r(0x0040, "inv_fault_lsb",   "Inverter Error Code LSB",     1.0,   "",    "uint16", "faults"),
    _r(0x0041, "inv_fault_msb",   "Inverter Error Code MSB",     1.0,   "",    "uint16", "faults"),
    _r(0x0043, "mgr_fault",       "Manager Fault Code",          1.0,   "",    "uint16", "faults"),
    _r(0x0044, "bms_fault_lsb",   "BMS Warning Code LSB",        1.0,   "",    "uint16", "faults"),
    _r(0x0045, "bms_fault_msb",   "BMS Warning Code MSB",        1.0,   "",    "uint16", "faults"),
    _r(0x0100, "modbus_pwr_ctrl", "Remote Power Control Mode",   1.0,   "",    "uint16", "control", "modbus_pwr_ctrl"),
    _r(0x011B, "target_soc",      "Target SOC (remote ctrl)",    1.0,   "%",   "uint16", "control"),
    _r(0x011F, "force_chg_flag",  "Battery Force Charge Flag",   1.0,   "",    "uint16", "control", "force_chg_flag"),
    _r(0x0120, "bms_relay_state", "BMS Relay State",             1.0,   "",    "uint16", "control", "bms_relay_state"),
    _r(0x011C, "shutdown",        "Shutdown State (X1)",         1.0,   "",    "uint16", "inverter", "yesno"),
    _r(0x011D, "microgrid",       "MicroGrid State (X1)",        1.0,   "",    "uint16", "inverter", "yesno"),
]

_SOLAX_HOLDING = [
    _r(0x0000, "sn_0",            "Serial Number chars 1-2",     1.0,   "",    "ascii",  "info"),
    _r(0x007D, "fw_dsp_minor",    "DSP Firmware Minor Ver",      1.0,   "",    "uint16", "firmware"),
    _r(0x007E, "hw_dsp",          "DSP Hardware Version",        1.0,   "",    "uint16", "firmware"),
    _r(0x007F, "fw_dsp_major",    "DSP Firmware Major Ver",      1.0,   "",    "uint16", "firmware"),
    _r(0x0080, "fw_arm_major",    "ARM Firmware Major Ver",      1.0,   "",    "uint16", "firmware"),
    _r(0x0082, "fw_modbus",       "Modbus RTU Firmware Ver",     1.0,   "",    "uint16", "firmware"),
    _r(0x0083, "fw_arm_minor",    "ARM Firmware Minor Ver",      1.0,   "",    "uint16", "firmware"),
    _r(0x0085, "rtc_sec",         "RTC Seconds",                 1.0,   "s",   "uint16", "rtc"),
    _r(0x0086, "rtc_min",         "RTC Minutes",                 1.0,   "min", "uint16", "rtc"),
    _r(0x0087, "rtc_hour",        "RTC Hours",                   1.0,   "h",   "uint16", "rtc"),
    _r(0x0088, "rtc_day",         "RTC Days",                    1.0,   "",    "uint16", "rtc"),
    _r(0x0089, "rtc_month",       "RTC Months",                  1.0,   "",    "uint16", "rtc"),
    _r(0x008A, "rtc_year",        "RTC Years",                   1.0,   "",    "uint16", "rtc"),
    _r(0x008B, "use_mode",        "Solar Charger Use Mode",      1.0,   "",    "uint16", "settings", "use_mode"),
    _r(0x008C, "manual_mode",     "Manual Mode Setting",         1.0,   "",    "uint16", "settings", "manual_mode"),
    _r(0x008D, "battery_type",    "Battery Type",                1.0,   "",    "uint16", "settings", "battery_type"),
    _r(0x008E, "float_voltage",   "Lead-acid Float Voltage",     0.1,   "V",   "uint16", "settings"),
    _r(0x008F, "discharge_cutv",  "Lead-acid Discharge Cut-off", 0.1,   "V",   "uint16", "settings"),
    _r(0x0090, "max_chg_i",       "Lead-acid Max Charge I",      0.1,   "A",   "uint16", "settings"),
    _r(0x0091, "max_dis_i",       "Lead-acid Max Discharge I",   0.1,   "A",   "uint16", "settings"),
    _r(0x001D, "safety_code",     "Safety Type Code",            1.0,   "",    "uint16", "settings"),
    _r(0x00AF, "modbus_address",  "Modbus RTU Slave Address",    1.0,   "",    "uint16", "settings"),
    _r(0x00B0, "modbus_baud",     "Modbus RTU Baud Rate Code",   1.0,   "",    "uint16", "settings", "baud_code"),
    _r(0x00BA, "inv_power_type",  "Inverter Power Type",         1.0,   "W",   "uint16", "info"),
    _r(0x00BB, "language",        "Display Language",            1.0,   "",    "uint16", "settings", "language"),
    _r(0x00BC, "enable_mppt",     "MPPT Enable",                 1.0,   "",    "uint16", "settings", "yesno"),
    _r(0x0104, "shadow_fix",      "Shadow Fix Function",         1.0,   "",    "uint16", "settings", "shadow_fix"),
    _r(0x0105, "machine_type",    "Machine Type (1=X1, 3=X3)",   1.0,   "",    "uint16", "info",     "machine_type"),
    _r(0x0108, "meter_function",  "Meter Function Enable",       1.0,   "",    "uint16", "settings", "yesno"),
    _r(0x0109, "meter1_id",       "Meter 1 Modbus ID",           1.0,   "",    "uint16", "settings"),
    _r(0x010B, "dir_meter_ct1",   "Direction Meter/CT 1",        1.0,   "",    "uint16", "settings", "direction"),
    _r(0x011C, "shutdown",        "Shutdown Enable (X1)",        1.0,   "",    "uint16", "control",  "yesno"),
    _r(0x011D, "microgrid",       "MicroGrid Enable (X1)",       1.0,   "",    "uint16", "control",  "yesno"),
    _r(0x011E, "selfuse_backup",  "Self-use Backup Enable",      1.0,   "",    "uint16", "control",  "yesno"),
    _r(0x011F, "backup_soc",      "Self-use Backup SOC",         1.0,   "%",   "uint16", "control"),
    _r(0x0130, "parallel_setting","Parallel Setting",            1.0,   "",    "uint16", "control",  "parallel"),
    _r(0x0131, "ext_gen_enable",  "External Generator Enable",   1.0,   "",    "uint16", "control",  "yesno"),
]

_SOLAX_LOOKUPS = {
    "run_mode": {
        0: "Waiting",   1: "Checking",  2: "Normal",
        3: "Fault",     4: "Perm Fault",5: "Update",
        6: "OG Wait",   7: "Off-grid",  8: "Self-test",
        9: "Idle",     10: "Standby",
    },
    "bdc_status":      {0: "Discharging", 1: "Charging", 2: "Stopped"},
    "grid_status":     {0: "On-Grid",     1: "Off-Grid"},
    "bms_connected":   {0: "Disconnected",1: "Connected"},
    "use_mode":        {0: "Self Use", 1: "Feed-in Priority", 2: "Back Up", 3: "Manual"},
    "manual_mode":     {0: "Stop", 1: "Force Charge", 2: "Force Discharge"},
    "battery_type":    {0: "Lead Acid", 1: "Lithium"},
    "modbus_pwr_ctrl": {0: "Disabled", 1: "Power Ctrl", 2: "Energy Ctrl", 3: "SOC Target"},
    "direction":       {0: "Positive", 1: "Negative"},
    "shadow_fix":      {0: "Off", 1: "Low", 2: "Middle", 3: "High"},
    "language":        {0: "English", 1: "German", 2: "French", 3: "Polish",
                        4: "Spanish", 5: "Portuguese", 6: "Italian"},
    "force_chg_flag":  {0: "No Action", 1: "Force Charge"},
    "bms_relay_state": {0: "OFF", 1: "ON"},
    "parallel":        {0: "Free", 1: "Master", 2: "Slave"},
    "yesno":           {0: "Disabled", 1: "Enabled"},
    "machine_type":    {1: "X1", 3: "X3"},
    "baud_code":       {0: 115200, 1: 57600, 2: 56000, 3: 38400,
                        4: 19200,  5: 14400,  6: 9600},
}

_SOLAX_INV_FAULTS = {
    0:  "TZ Protect",    1:  "Grid Lost",      2:  "Grid Volt",
    3:  "Grid Freq",     4:  "PV Volt",         5:  "Bus Volt",
    6:  "Bat Volt",      7:  "AC10min Volt",    8:  "DCI OCP",
    10: "SW OCP",        11: "RC OCP",          12: "Isolation",
    13: "Temp Over",     14: "BatConnDir",      15: "Missed CT",
    16: "Off-grid OL",   17: "Overload",        18: "PV ConnDir",
    19: "Bat Power Low", 20: "Low Temp",        22: "Charger Relay",
    23: "BMS Lost",      24: "Inner Comm",      25: "Fan Fault",
    26: "Earth Relay",   27: "INV EEPROM",      28: "RCD Fault",
    29: "Off-grid Relay",30: "Grid Relay",      31: "Other Device",
}

_SOLAX_BMS_FAULTS = {
    0:  "External Err",   1:  "Internal Err",   2:  "OverVoltage",
    3:  "LowVoltage",     4:  "ChargeOCP",       5:  "DischargeOCP",
    6:  "TempHigh",       7:  "TempLow",         8:  "CellImbalance",
    9:  "HW Protect",    10:  "Circuit Fault",  11:  "ISO Fault",
    12: "VolSen Fault",  13:  "TempSen Fault",  14:  "CurSen Fault",
    15: "Relay Fault",   16:  "Type Unmatch",   17:  "Ver Unmatch",
    18: "MFR Unmatch",   19:  "SW Unmatch",     20:  "M&S Unmatch",
    21: "CR NoRespond",  22:  "SW Protect",     24:  "SelfcheckErr",
    25: "TempdiffErr",   26:  "BreakFault",     27:  "Flash Fault",
    28: "Precharge Fault",29: "AirSwitch Break",
}

_SOLAX_DEMO: dict[int, int] = {
    0x0000: 2300, 0x0001: 150,    0x0002: 345,   0x0007: 5001, 0x001A: 0,
    0x0003: 3850, 0x0004: 3720,   0x0005: 85,    0x0006: 60,
    0x000A: 3270, 0x000B: 2232,
    0x0008: 42,   0x0009: 2,      0x001B: 2,
    0x0014: 520,  0x0015: 80,     0x0016: 416,   0x0018: 28,
    0x001C: 72,   0x0019: 1,      0x0017: 1,
    0x0024: 250,  0x0025: 250,
    0x00BA: 285,  0x00BB: 272,
    0x00BC: 3650, 0x00BD: 3620,
    0x00BE: 72,   0x00BF: 98,
    0x004C: 0,    0x004D: 0,      0x004E: 0,     0x004F: 0,
    0x0046: 0xFEE0, 0x0047: 0xFFFF,              # -288 W importing
    0x0050: 142,  0x0096: 187,    0x0020: 55,    0x0023: 70,
    0x0091: 45,   0x0090: 0,
    0x003E: 0,    0x003F: 0,      0x0040: 0,     0x0041: 0,
    0x0043: 0,    0x0044: 0,      0x0045: 0,
    0x0100: 0,    0x011B: 0,      0x011F: 0,     0x0120: 1,
    0x011C: 0,    0x011D: 0,
}


def _solax_build_snapshot(words: dict[int, int]) -> dict:
    """Returns a flat dict of decoded values for the SolaX dashboard tab."""
    ir = {r.addr: r for r in _SOLAX_INPUT}

    def val(name: str) -> Optional[float]:
        reg = next((r for r in ir.values() if r.name == name), None)
        return decode_reg(words, reg) if reg else None

    def raw(addr: int) -> int:
        return words.get(addr, 0)

    feedin_lo, feedin_hi = raw(0x0046), raw(0x0047)
    feedin = round(as_int32(feedin_lo, feedin_hi), 0) if (feedin_lo or feedin_hi) else None

    return {
        "timestamp":       datetime.now(),
        "grid_voltage":    val("grid_voltage"),
        "grid_current":    val("grid_current"),
        "grid_power":      val("grid_power"),
        "grid_frequency":  val("grid_frequency"),
        "grid_status":     raw(0x001A),
        "feedin_power":    feedin,
        "pv1_voltage":     val("pv1_voltage"),
        "pv1_current":     val("pv1_current"),
        "pv1_power":       val("pv1_power"),
        "pv2_voltage":     val("pv2_voltage"),
        "pv2_current":     val("pv2_current"),
        "pv2_power":       val("pv2_power"),
        "pv_total":        (val("pv1_power") or 0) + (val("pv2_power") or 0),
        "bat_voltage":     val("bat_voltage"),
        "bat_current":     val("bat_current"),
        "bat_power":       val("bat_power"),
        "bat_temp":        val("bat_temp"),
        "soc":             raw(0x001C),
        "bdc_status":      raw(0x0019),
        "bms_connected":   raw(0x0017),
        "bms_soc":         val("bms_soc"),
        "bms_soh":         val("bms_soh"),
        "cell_v_high":     val("cell_v_high"),
        "cell_v_low":      val("cell_v_low"),
        "inv_temp":        val("inv_temp"),
        "run_mode":        raw(0x0009),
        "yield_today":     val("yield_today"),
        "solar_today":     val("solar_today"),
        "bat_chg_today":   val("bat_chg_today"),
        "echarge_today":   val("echarge_today"),
        "offgrid_today":   val("offgrid_today"),
        "offgrid_voltage": val("offgrid_voltage"),
        "offgrid_power":   val("offgrid_power"),
        "inv_fault_word":  (raw(0x0041) << 16) | raw(0x0040),
        "bms_fault_word":  (raw(0x0045) << 16) | raw(0x0044),
        "pcs_fault":       raw(0x003E),
        "mgr_fault":       raw(0x0043),
        "_words":          words,
    }


# ── Build the SolaX profile ────────────────────────────────────────────────
SOLAX_X1 = InverterProfile(
    name           = "SolaX X1 Hybrid G4",
    input_regs     = {r.addr: r for r in _SOLAX_INPUT},
    holding_regs   = {r.addr: r for r in _SOLAX_HOLDING},
    input_chunks   = [
        (0x0000, 0x0030), (0x003E, 0x000A), (0x004C, 0x0008),
        (0x0050, 0x0002), (0x008E, 0x0012), (0x00B8, 0x0010),
        (0x0100, 0x0022), (0x011A, 0x000A),
    ],
    holding_chunks = [
        (0x0000, 0x0020), (0x007D, 0x0040),
        (0x00AF, 0x0020), (0x0100, 0x0040), (0x011A, 0x0020),
    ],
    lookups        = _SOLAX_LOOKUPS,
    fault_bit_maps = {
        "Inverter Faults": _SOLAX_INV_FAULTS,
        "BMS Warnings":    _SOLAX_BMS_FAULTS,
    },
    demo_words     = _SOLAX_DEMO,
    build_snapshot = _solax_build_snapshot,
)

# ── Profile registry — add new inverters here ──────────────────────────────
# To add a new inverter:
#   1. Define its RegDef lists, lookup tables, fault maps, and chunk ranges
#   2. Create an InverterProfile instance
#   3. Add it to PROFILES below — the UI picks it up automatically
PROFILES: dict[str, Optional[InverterProfile]] = {
    "Generic (raw Modbus)": None,
    "SolaX X1 Hybrid G4":  SOLAX_X1,
    # "Growatt SPH":         GROWATT_SPH,
    # "Deye SUN-*":          DEYE_SUN,
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
  /* Metric cards */
  [data-testid="metric-container"] {
      background: #1c1c1c;
      border: 1px solid #2e2e2e;
      border-radius: 6px;
      padding: 12px 16px;
  }
  [data-testid="stMetricLabel"] { font-size: 0.72rem; color: #888; text-transform: uppercase; letter-spacing: 0.05em; }
  [data-testid="stMetricValue"] { font-size: 1.3rem; font-weight: 600; color: #e0e0e0; }
  [data-testid="stMetricDelta"] { font-size: 0.72rem; }

  /* Tabs */
  [data-testid="stTabs"] button[role="tab"] { font-weight: 500; font-size: 0.85rem; }

  /* Sidebar */
  [data-testid="stSidebar"] { background: #141414; }

  /* Section headers */
  .section-header {
      font-size: 0.7rem;
      font-weight: 600;
      letter-spacing: 0.15em;
      text-transform: uppercase;
      color: #555;
      margin: 20px 0 10px 0;
      padding-bottom: 5px;
      border-bottom: 1px solid #2a2a2a;
  }

  /* Status badges */
  .badge-ok   { background: transparent; color: #aaa;
                border: 1px solid #3a3a3a; border-radius: 4px;
                padding: 2px 10px; font-size: 0.78rem; }
  .badge-warn { background: transparent; color: #c8922a;
                border: 1px solid #c8922a; border-radius: 4px;
                padding: 2px 10px; font-size: 0.78rem; }
  .badge-err  { background: transparent; color: #b85c5c;
                border: 1px solid #b85c5c; border-radius: 4px;
                padding: 2px 10px; font-size: 0.78rem; }

  /* SOC bar */
  .soc-wrap { background: #222; border-radius: 4px; height: 18px;
              margin: 4px 0 16px 0; overflow: hidden; }
  .soc-fill { height: 100%; border-radius: 4px; display: flex;
              align-items: center; justify-content: center;
              color: #ccc; font-size: 11px; font-weight: 600;
              transition: width 0.5s ease; }
</style>
""", unsafe_allow_html=True)


# ── Small UI helpers ───────────────────────────────────────────────────────

def _section(label: str):
    st.markdown(f'<div class="section-header">{label}</div>', unsafe_allow_html=True)


def _badge(text: str, level: str = "ok"):
    cls = {"ok": "badge-ok", "warn": "badge-warn", "err": "badge-err"}.get(level, "badge-ok")
    st.markdown(f'<span class="{cls}">{text}</span>', unsafe_allow_html=True)


def _soc_bar(pct: int):
    pct = max(0, min(100, int(pct)))
    # Monochrome: light fill when high, dims as it drops
    if pct > 50:
        color = "#555"
    elif pct > 20:
        color = "#4a4a4a"
    else:
        color = "#7a3a3a"   # only hint of color at critically low SOC
    st.markdown(
        f'<div class="soc-wrap"><div class="soc-fill" style="width:{pct}%;background:{color};">'
        f'{pct}%</div></div>',
        unsafe_allow_html=True,
    )


def _fault_expander(title: str, word32: int, bit_map: dict[int, str]):
    faults = active_faults(word32, bit_map)
    if faults:
        with st.expander(f"{title} — {len(faults)} active fault(s)"):
            for f in faults:
                st.markdown(f"- {f}")
    else:
        _badge(f"{title}: OK", "ok")


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

    if profile:
        st.caption(f"Profile: {profile.name}")
    else:
        st.caption("No profile selected — raw mode only.")

    st.divider()

    mode = st.radio(
        "Connection",
        ["Live Serial", "Demo / Offline"],
        help="Demo mode uses the profile's built-in sample data.",
    )
    live_mode = mode == "Live Serial"

    if live_mode and not SERIAL_AVAILABLE:
        st.error("pyserial not installed.\npip install pyserial")
        live_mode = False

    if live_mode:
        ports = [p.device for p in serial.tools.list_ports.comports()] or ["COM4"]
        default_port = "COM4" if "COM4" in ports else ports[0]
        port     = st.selectbox("Port",     ports, index=ports.index(default_port))
        baud     = st.selectbox("Baud",     list(VALID_BAUDS), index=list(VALID_BAUDS).index(9600))
        slave_id = st.number_input("Slave ID", 1, 247, 1)
        timeout  = st.slider("Timeout (s)", 0.5, 5.0, 1.0, 0.5)
        retries  = st.number_input("Retries", 0, 5, 2)
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
        fb_val = st.number_input("Count (read) / Value (write)", min_value=0, max_value=65535, value=1)
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
        log_rows = []
        prog = st.progress(0)
        last_resp = None

        for i in range(int(fb_rpt)):
            frame, resp = send_frame(
                port, int(baud), int(slave_id),
                fc_b, reg_b, int(fb_val),
                timeout=float(timeout), retries=0, gap=0.12,
            )
            if resp:
                ok = crc_valid(resp, fc_b)
                last_resp = resp
                log_rows.append({
                    "#":        i + 1,
                    "Sent":     frame.hex(" ").upper(),
                    "Response": resp.hex(" ").upper(),
                    "CRC":      "OK" if ok else "FAIL",
                    "Bytes":    len(resp),
                })
            else:
                log_rows.append({
                    "#": i + 1, "Sent": frame.hex(" ").upper(),
                    "Response": "NO RESPONSE", "CRC": "—", "Bytes": 0,
                })
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

    dec_source = st.radio(
        "Source",
        ["Last Frame Builder response", "Paste hex"],
        horizontal=True,
    )

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

    elif dec_source == "Paste hex":
        with st.expander("Example responses"):
            st.code("Input regs:      01 04 06 00 00 00 00 00 00 60 93")
            st.code("Holding regs:    01 03 0E 48 34 37 35 32 32 5A 48 45 4E 47 57 45 4E 63 26")
            st.code("Write single:    01 06 00 1F 00 00 48 0A")

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
                ref_map = (profile.input_regs if fc_r == FC_READ_INPUT else profile.holding_regs) if profile else {}

                rows = []
                for idx, w in enumerate(words):
                    addr = (start_reg + idx) if start_reg is not None else idx
                    row: dict = {
                        "Offset":  idx,
                        "Address": f"0x{addr:04X}" if start_reg is not None else "—",
                        "uint16":  w,
                        "int16":   as_int16(w),
                        "Hex":     f"0x{w:04X}",
                        "x0.1":    round(w * 0.1, 1),
                        "x0.01":   round(w * 0.01, 2),
                    }
                    if start_reg is not None and addr in ref_map and profile:
                        reg    = ref_map[addr]
                        scaled = decode_reg({addr: w}, reg)
                        label  = resolve_label(profile, reg, w)
                        row.update({
                            "Name":        reg.name,
                            "Category":    reg.category,
                            "Scaled":      _fmt(scaled, reg.unit),
                            "Value":       label or _fmt(scaled, reg.unit),
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

    if profile.build_snapshot is None:
        st.warning(f"Profile '{profile.name}' does not define a dashboard.")
        st.stop()

    dc1, dc2, dc3 = st.columns([2, 1, 1])
    with dc1:
        poll_interval = st.slider("Refresh interval (s)", 3, 120, 10, label_visibility="collapsed")
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
            st.warning("No demo data defined for this profile.")
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
    h["soc"].append(snap.get("soc", 0))

    # Status strip
    _section("Status")
    s1, s2, s3, s4, s5, s6 = st.columns(6)
    s1.metric("Run Mode",   lk.get("run_mode",    {}).get(snap["run_mode"],    str(snap["run_mode"])))
    s2.metric("Grid",       lk.get("grid_status", {}).get(snap["grid_status"], "—"))
    s3.metric("Battery",    lk.get("bdc_status",  {}).get(snap["bdc_status"],  "—"))
    s4.metric("Inv Temp",   _fmt(snap.get("inv_temp"), "°C"))
    s5.metric("BMS",        lk.get("bms_connected",{}).get(snap["bms_connected"],"—"))
    s6.metric("Last Poll",  snap["timestamp"].strftime("%H:%M:%S"))

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
        st.caption("Combined")
        st.metric("Total Power", _fmt(snap.get("pv_total"),    "W"))
        st.metric("Solar Today", _fmt(snap.get("solar_today"), "kWh"))

    st.divider()

    # Battery
    _section("Battery")
    b1, b2, b3, b4, b5 = st.columns(5)
    b1.metric("Voltage",     _fmt(snap.get("bat_voltage"), "V"))
    b2.metric("Current",     _fmt(snap.get("bat_current"), "A"))
    b3.metric("Power",       _fmt(snap.get("bat_power"),   "W"))
    b4.metric("Temperature", _fmt(snap.get("bat_temp"),   "°C"))
    b5.metric("SOC",         _fmt(snap.get("soc"),         "%"))
    _soc_bar(snap.get("soc", 0))

    bc1, bc2 = st.columns(2)
    with bc1:
        st.metric("BMS SOC", _fmt(snap.get("bms_soc"), "%"))
        st.metric("BMS SOH", _fmt(snap.get("bms_soh"), "%"))
    with bc2:
        cvh = snap.get("cell_v_high")
        cvl = snap.get("cell_v_low")
        st.metric("Cell High", _fmt(cvh, "V"))
        st.metric("Cell Low",  _fmt(cvl, "V"))
        if cvh is not None and cvl is not None:
            spread = round(cvh - cvl, 3)
            st.metric("Spread", _fmt(spread, "V"),
                      delta="OK" if spread < 0.05 else "High",
                      delta_color="normal" if spread < 0.05 else "inverse")

    st.divider()

    # Grid
    _section("Grid")
    g1, g2, g3, g4, g5 = st.columns(5)
    g1.metric("Voltage",   _fmt(snap.get("grid_voltage"),   "V"))
    g2.metric("Current",   _fmt(snap.get("grid_current"),   "A"))
    g3.metric("Power",     _fmt(snap.get("grid_power"),     "W"))
    g4.metric("Frequency", _fmt(snap.get("grid_frequency"), "Hz"))
    fi = snap.get("feedin_power")
    if fi is not None:
        fi_label = f"{abs(fi)} W  ({'Export' if fi >= 0 else 'Import'})"
    else:
        fi_label = "—"
    g5.metric("Feed-in", fi_label)

    st.divider()

    # Energy today
    _section("Energy Today")
    e1, e2, e3, e4 = st.columns(4)
    e1.metric("Yield (AC)",   _fmt(snap.get("yield_today"),   "kWh"))
    e2.metric("Solar",        _fmt(snap.get("solar_today"),   "kWh"))
    e3.metric("Bat Charged",  _fmt(snap.get("echarge_today"), "kWh"))
    e4.metric("Off-grid",     _fmt(snap.get("offgrid_today"), "kWh"))

    st.divider()

    # Faults
    _section("Fault Status")
    fault_maps  = profile.fault_bit_maps
    fault_cols  = st.columns(max(len(fault_maps), 1))
    for col, (title, bmap) in zip(fault_cols, fault_maps.items()):
        key = "inv_fault_word" if "Inv" in title else "bms_fault_word"
        with col:
            _fault_expander(title, snap.get(key, 0), bmap)

    pf1, pf2 = st.columns(2)
    with pf1:
        pcs = snap.get("pcs_fault", 0)
        _badge(f"PCS: 0x{pcs:04X}" if pcs else "PCS: OK", "err" if pcs else "ok")
    with pf2:
        mgr = snap.get("mgr_fault", 0)
        _badge(f"Manager: 0x{mgr:04X}" if mgr else "Manager: OK", "err" if mgr else "ok")

    st.divider()

    # Trend charts
    if len(h["ts"]) > 1:
        _section("Trend (last 60 polls)")
        df_trend = pd.DataFrame({
            "Time":         list(h["ts"]),
            "PV (W)":       list(h["pv_power"]),
            "Battery (W)":  list(h["bat_power"]),
            "Grid (W)":     list(h["grid_power"]),
            "SOC (%)":      list(h["soc"]),
        }).set_index("Time")
        ch1, ch2 = st.columns(2)
        ch1.line_chart(df_trend[["PV (W)", "Battery (W)", "Grid (W)"]], height=200)
        ch2.line_chart(df_trend[["SOC (%)"]], height=200)

    # Raw register dump
    words_raw = snap.get("_words", {})
    if words_raw:
        with st.expander("Raw register dump"):
            raw_df = pd.DataFrame([
                {"Addr": f"0x{k:04X}", "Dec": k,
                 "uint16": v, "int16": as_int16(v), "Hex": f"0x{v:04X}"}
                for k, v in sorted(words_raw.items())
            ])
            st.dataframe(raw_df, use_container_width=True, hide_index=True)


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  TAB 4 — BULK SCAN                                                     ║
# ╚══════════════════════════════════════════════════════════════════════════╝
with tab4:
    st.markdown("#### Bulk Register Scan")
    st.caption(
        "Scan a register range in chunks. "
        "Useful for discovering unknown registers or verifying a full map. "
        "Results are decoded against the selected profile where possible."
    )

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
            st.error("Invalid register address.")
            st.stop()

        if end_a <= start_a:
            st.error("End must be greater than start.")
            st.stop()

        chunks = [
            (addr, min(int(scan_chunk), end_a - addr + 1))
            for addr in range(start_a, end_a + 1, int(scan_chunk))
        ]

        ref_map = {}
        if profile:
            ref_map = profile.input_regs if scan_fc == FC_READ_INPUT else profile.holding_regs

        all_words: dict[int, int] = {}
        prog = st.progress(0)

        for i, (chunk_start, chunk_count) in enumerate(chunks):
            _, resp = send_frame(
                port, int(baud), int(slave_id),
                scan_fc, chunk_start, chunk_count,
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
                "Address": f"0x{addr:04X}",
                "uint16":  w,
                "int16":   as_int16(w),
                "Hex":     f"0x{w:04X}",
                "x0.1":    round(w * 0.1, 1),
                "x0.01":   round(w * 0.01, 2),
            }
            if addr in ref_map and profile:
                reg    = ref_map[addr]
                scaled = decode_reg({addr: w}, reg)
                label  = resolve_label(profile, reg, w)
                row.update({
                    "Name":        reg.name,
                    "Category":    reg.category,
                    "Scaled":      _fmt(scaled, reg.unit),
                    "Value":       label or _fmt(scaled, reg.unit),
                    "Description": reg.desc,
                })
            rows.append(row)

        df_scan = pd.DataFrame(rows)
        st.markdown(f"{len(df_scan)} registers  (0x{start_a:04X} – 0x{end_a:04X}, FC 0x{scan_fc:02X})")
        st.dataframe(df_scan, use_container_width=True, hide_index=True)


# ── Footer ─────────────────────────────────────────────────────────────────
st.divider()
st.caption("Modbus RTU Inspector  |  Min frame interval: 1 s  |  EEprom registers have limited write cycles")
