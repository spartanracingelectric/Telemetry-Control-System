# Telemetry Control System (TCS)

Telemetry Control System (TCS) is a system designed to enable real-time communication and control between a car and a laptop using two ESP32 devices connected via LoRa. The project includes embedded firmware for both devices and a Python application for monitoring and control.

---

## Table of Contents

* [Overview](#overview)
* [Features](#features)
* [Team / Contributors](#team--contributors)
* [Repository Structure](#repository-structure)
* [Prerequisites](#prerequisites)
* [Building & Flashing Firmware](#building--flashing-firmware)
* [Running the Python Application](#running-the-python-application)
* [Workflow](#workflow)
* [Contact](#contact)

---

## Overview

The Telemetry Control System consists of:

* **ESP32 on the car (Master)** — collects data and sends it over LoRa.
* **ESP32 on the laptop (Mini)** — receives data from the car and can send commands back.
* **Python application** — interfaces with the Master ESP32 for monitoring, logging, and control.

This setup allows real-time telemetry, testing, and demonstration of embedded firmware communication over LoRa.

---

## Features

* Real-time bidirectional communication between car and laptop via LoRa.
* Firmware for both Master and Mini ESP32 devices.
* Python application for monitoring, control, and data visualization.
* Modular and extensible design for future enhancements.

---

## Team / Contributors

| Name        | Role / Responsibility                                         |
| ----------- | ------------------------------------------------------------- |
| **Shinika** | Team lead; TCS Mini firmware development and LoRa integration |
| **Jordan**  | TCS Master firmware development                               |
| **Hannah**  | Python application development                                |
| **Hemanth** | Hardware development support                                  |

---

## Repository Structure

* `TCS-Mini/` — Firmware for the Master ESP32 on the laptop.
* `TCS-Master/` — Firmware for the Mini ESP32 on the car.
* `Software/` — Python application for monitoring and control.

---

## Prerequisites

### Firmware (ESP32)

* ESP-IDF (version compatible with your ESP32). Installable through VSCode. 
* Dependencies (on Linux/Ubuntu):

```bash
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

* A USB cable to connect the ESP32 devices to your computer.

### Python Application

* Python > 3.12
* Python dependencies (install via `requirements.txt`)

---

## Building & Flashing Firmware

1. **Clone the repository**:

```bash
git clone https://github.com/spartanracingelectric/Telemetry-Control-System.git
```

2. **Set up ESP-IDF**:

```bash
cd ~/esp
git clone -b <version> --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
. ./export.sh
```

3. **Build firmware**:

* For Master ESP32:

```bash
cd TCS-Master
idf.py build
```

* For Mini ESP32:

```bash
cd TCS-Mini
idf.py build
```

4. **Flash firmware**:

```bash
idf.py -p /dev/ttyUSB0 flash
```

Replace `/dev/ttyUSB0` with your device port.

5. **Monitor output**:

```bash
idf.py monitor
```

---

## Running the Python Application

1. Navigate to the Python application folder:

```bash
cd Software
```

2. (Optional) Create a virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate
```

3. Install dependencies:

```bash
pip install -r requirements.txt
```

4. Run the application:

```bash
python main.py
```

---

## Workflow

1. Flash firmware to both Master (car) and Mini (laptop) ESP32 devices.
2. Connect hardware (sensors, CAN, LoRa antennas) as needed.
3. Boot the devices; Master sends telemetry data to Mini.
4. Run the Python application to monitor data, log telemetry, and send commands.

---

## Contact

* **Project Lead**: Shinika
