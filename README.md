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
* [CI/CD Pipeline](#cicd-pipeline)
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

* Real-time bidirectional communication between car and laptop via LoRa
* Firmware for both Master and Mini ESP32 devices
* Python application for monitoring, control, and data visualization
* Modular and extensible design for future enhancements

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

### `Software/` — Python Application

Contains the telemetry dashboard and UART communication layer.

#### Files

* **`tcs_app.py`**
  Main GUI dashboard using DearPyGUI.
  Handles UI layout, threading, live telemetry display, command sending, etc.

* **`uart_data_read.py`**
  Serial interface module.
  Handles UART reads, packet parsing, and communication with the GUI layer.

---

### `TCS-Mini/` — ESP32 Firmware (Laptop Side)

Mini = laptop-side ESP32 that receives telemetry via LoRa and can send commands.

Includes:

* **LoRa Modules:**
  `lora_app.c`, `lora_app.h`

* **UART Modules:**
  `uart_app.c`, `uart_app.h`

* **Networking / Web:**
  `wifi_app.c`, `wifi_app.h`, `web_app.c`, `web_app.h`, `http_server.c`, `http_server.h`

* **Common:**
  `tasks_common.h`, `main.c`, `CMakeLists.txt`, `Kconfig.projbuild`, `idf_component.yml`

* **`webpage/`** — Static web assets

---

### `TCS-Master/` — ESP32 Firmware (Car Side)

Master = car-side ESP32 responsible for collecting sensor data and transmitting telemetry.

#### Files

* **`main.c`, `main.h`**
  Firmware entry point.

* **`telemetry.c`, `telemetry.h`**
  Builds telemetry packets from all available sensors.

* **`lora_handler.c`, `lora_handler.h`**
  Handles LoRa transmission.

* **`can_lora.c`, `can_lora.h`**
  CAN bus reading and telemetry conversion.

* **`crash_imu.c`, `crash_imu.h`**
  IMU crash/acceleration detection.

* **`humidity.c`, `humidity.h`**
  Humidity/environment sensor.

* **`rgb_ledc_controller.c`, `rgb_ledc_controller.h`**
  LED status indicator logic.

* **Build Files:**
  `CMakeLists.txt`, `component.mk`, `Kconfig.projbuild`

---

## Prerequisites

### Firmware (ESP32)

```bash
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

### Python

```bash
pip install -r requirements.txt
```

---

## Building & Flashing Firmware

```bash
git clone https://github.com/spartanracingelectric/Telemetry-Control-System.git
```

### Build

```bash
cd TCS-Master
idf.py build

cd TCS-Mini
idf.py build
```

### Flash

```bash
idf.py -p /dev/ttyUSB0 flash
```

---

## Running the Python Application

```bash
cd Software
python tcs_app.py
```

---

## Workflow

1. Flash both ESP32 devices.
2. Power the car-side Master.
3. Power the laptop-side Mini.
4. Launch the Python dashboard.

---

## CI/CD Pipeline

### **1. Python Syntax & Build Check**

* Checks syntax for `tcs_app.py` and `uart_data_read.py`
* Ensures imports load properly
* Fails on any syntax or import error

### **2. ESP-IDF Build Pipeline**

* Installs ESP-IDF toolchain
* Builds TCS-Master and TCS-Mini
* Fails on any compile or configuration error

---

## Contact

**Project Lead:** Shinika
