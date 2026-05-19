# ⚡ SolaX X1 Hybrid G4 — Modbus RTU Dashboard

A Streamlit-based real-time monitoring dashboard for the **SolaX X1 Hybrid G4** inverter using the Modbus RTU protocol (V3.21, single-phase).

![Python](https://img.shields.io/badge/Python-3.9%2B-blue)
![Streamlit](https://img.shields.io/badge/Streamlit-1.35%2B-red)
![License](https://img.shields.io/badge/License-MIT-green)

---

## Features

| Tab | Description |
|-----|-------------|
| 📊 Live Dashboard | Real-time polling — metrics, SOC bar, trend charts, fault status |
| 🔍 Data Interpreter | Full register decode table with scaling, units, and friendly labels |
| 🛠 Manual Frame Test | Custom Modbus frame builder and sender |
| 🧩 Hex / Excel Decoder | Paste raw hex or upload an Excel export to decode offline |

Works **with or without hardware** — a built-in demo mode simulates a live X1 system.

---

## Requirements

- Python **3.9 or later**
- A SolaX X1 Hybrid G4 inverter connected via **RS-485 → USB adapter** *(optional — demo mode works without hardware)*

---

## Quick Start

### 1. Clone the repository

```bash
git clone https://github.com/YOUR_USERNAME/solax-x1-dashboard.git
cd solax-x1-dashboard
```

### 2. Create and activate a virtual environment (recommended)

```bash
# macOS / Linux
python3 -m venv .venv
source .venv/bin/activate

# Windows
python -m venv .venv
.venv\Scripts\activate
```

### 3. Install dependencies

```bash
pip install -r requirements.txt
```

### 4. Run the app

```bash
streamlit run solax_x1_gui.py
```

The browser will open automatically at `http://localhost:8501`.

---

## Hardware Setup (Live Serial Mode)

| Item | Detail |
|------|--------|
| Interface | RS-485 to USB adapter |
| Inverter port | COM port on X1 (9-pin RJ45 or terminal block — see inverter manual) |
| Default baud rate | **9600** baud (configurable in inverter settings) |
| Default slave ID | **1** |
| Wiring | A(+) → A(+), B(−) → B(−), GND → GND |

> ⚠️ **EEprom registers** (marked ★ in the protocol spec) have **limited write cycles**.  
> Use the Write Single function sparingly — do not loop writes.

---

## Project Structure

```
solax-x1-dashboard/
├── solax_x1_gui.py      # Main Streamlit application
├── requirements.txt     # Python dependencies
├── .gitignore           # Files to exclude from git
└── README.md            # This file
```

---

## Configuration

All connection parameters are set in the **sidebar** at runtime — no config file required:

| Setting | Default | Notes |
|---------|---------|-------|
| Serial Port | `COM4` | Auto-detected on your system |
| Baud Rate | `9600` | Must match inverter setting |
| Slave ID | `1` | Modbus address of the inverter |
| Response Timeout | `1.0 s` | Increase for long cable runs |
| Auto-refresh interval | `10 s` | Minimum spec: 1 s |

---

## Modbus Protocol Notes

- **Function code 0x04** — Read Input Registers (live sensor data)
- **Function code 0x03** — Read Holding Registers (settings, firmware info)
- **Function code 0x06** — Write Single Register (remote control)
- Minimum inter-frame gap: **> 100 ms**
- Minimum poll interval: **1 s**
- CRC: Modbus CRC-16 (little-endian appended)

---

## Dependencies

| Package | Purpose |
|---------|---------|
| `streamlit` | Web UI framework |
| `pyserial` | RS-485/USB serial communication |
| `crcmod` | Modbus CRC-16 calculation |
| `pandas` | Data tables and CSV export |
| `openpyxl` | Excel file upload/parsing |

---

## License

MIT — see [LICENSE](LICENSE) for details.

---

## Disclaimer

This project is **not affiliated with SolaX Power**. Use at your own risk.  
Always follow your inverter's official documentation and local electrical codes.
