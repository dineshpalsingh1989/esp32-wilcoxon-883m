# Esp32-Wilcoxon-883M
Firmware for Industrial IoT Vibration Monitoring using ESP32 and Wilcoxon 883M sensors.
# 🏭 Industrial IoT Vibration Monitor (ESP32 + Wilcoxon 883M)

![License](https://img.shields.io/badge/license-MIT-green)
![Platform](https://img.shields.io/badge/platform-ESP32-blue)
![Framework](https://img.shields.io/badge/framework-Arduino-orange)
![Status](https://img.shields.io/badge/status-Production%20Ready-success)

> **A professional-grade IoT gateway firmware for monitoring industrial machinery vibration using the Wilcoxon 883M (RS485) sensor and the Cytron Maker Feather AIoT S3.**

---

## 📋 Project Overview

This firmware transforms a standard Modbus RTU vibration sensor into a modern **Industrial IoT Node**. It provides real-time monitoring of critical machinery health metrics (Velocity, Acceleration, Displacement) via a web dashboard, MQTT, and Modbus TCP.

### ✨ Key Features
* **📊 Real-Time Web Dashboard:** Hosted directly on the ESP32. Visualizes live metrics and high-frequency waveform/spectrum graphs using Chart.js.
* **☁️ MQTT Integration:** Publishes health status, metrics, and raw data chunks to any MQTT broker (TLS supported).
* **🔗 Modbus TCP Gateway:** Acts as a bridge, allowing SCADA systems (Ignition, Node-RED) to query the sensor over Wi-Fi.
* **🚨 Smart Alerts:** Local buzzer alarms and LED indicators for Wi-Fi loss or sensor errors.
* **💾 CSV Export:** Download internal metric logs directly from the browser.
* **🛡️ Robust Design:** Dual-core watchdog timers and auto-recovery logic for 24/7 operation.

---

## 🛠️ Hardware Requirements

| Component | Description | Notes |
| :--- | :--- | :--- |
| **MCU** | [Cytron Maker Feather AIoT S3](https://www.cytron.io/) | ESP32-S3 with built-in RGB LED & Buzzer |
| **Sensor** | [Wilcoxon 883M](https://wilcoxon.com/) | Digital Vibration Sensor (RS485 Modbus RTU) |
| **Power** | 12V-24V DC PSU | Required to power the 883M sensor |
| **Module** | RS485-to-TTL UART | For ESP32 to communicate with the sensor |

### 🔌 Pin Configuration
| ESP32 Pin | Function |
| :--- | :--- |
| **GPIO 16** | RS485 RX |
| **GPIO 15** | RS485 TX |
| **GPIO 12** | Buzzer (Alarm) |
| **GPIO 3** | User Button (Hold 5s for AP Mode) |
| **GPIO 46** | RGB Status LED |

---

## 🚀 Installation & Setup

### 1. Flash the Firmware
You can use **PlatformIO** (recommended) or Arduino IDE.
1.  Clone this repository.
2.  Open the folder in VS Code with the PlatformIO extension installed.
3.  Connect your ESP32 via USB.
4.  Click **Upload** (➡️).

### 2. Connect to Wi-Fi
1.  On first boot, the device will create a Wi-Fi Access Point:
    * **SSID:** `ILA_Sensor_AP`
    * **Password:** `12345678`
2.  Connect your phone/laptop to this network.
3.  Open a browser to `http://192.168.4.1`.
4.  Go to **Settings > Wi-Fi** to scan and save your local network credentials.

---

## 📡 API & Communication

### 🟢 Status LED Codes
* 🟢 **Green:** System Healthy (Idle)
* 🟡 **Yellow (Blinking):** Sensor Busy (Downloading Waveform)
* 🔴 **Red:** Sensor Error / Timeout
* ⚪ **White:** Wi-Fi Disconnected
* 🔵 **Cyan:** Metrics Recording Active

### ☁️ MQTT Topics
The device publishes JSON data to the following topics (default base: `883M`):

| Topic | Direction | Description |
| :--- | :--- | :--- |
| `883M/status` | Out | Device health, uptime, and identity |
| `883M` | Out | Main metrics payload and waveform chunks |
| `883M/command` | In | Remote control (Reboot, Trigger Capture) |

**Example Command (Trigger Capture):**
```json
{
  "command": "trigger_capture",
  "axis": "X"
}
