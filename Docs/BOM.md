# 📦 Bill of Materials (BOM) – ADAS Robotic Vehicle Prototype

This document outlines all the electronic components, sensors, actuators, and power elements used in the ADAS robotic vehicle prototype.

---

## 🧠 Microcontrollers & Processing Units

| Item                  | Quantity | Description                                          |
|-----------------------|----------|------------------------------------------------------|
| Raspberry Pi 4        | 1        | Main processor for computer vision (TSDR)            |
| ESP32 DevKit          | 1        | Main embedded controller for low-level functions     |

---

## 🎯 Sensors

| Sensor/Module            | Quantity | Functionality                                      |
|--------------------------|----------|----------------------------------------------------|
| HC-SR04 Ultrasonic Sensor| 3        | Distance sensing for AEB, ACC, and BSW             |
| IR Sensor Module         | 2        | Lane detection for LKA                             |
| LDR Photocell            | 1        | Ambient light sensing for ALS                      |
| Raspberry Pi Camera V1.3 | 1        | Visual input for traffic sign detection (TSDR)     |

---

## ⚙️ Actuators & Motors

| Item               | Quantity | Description                                 |
|--------------------|----------|---------------------------------------------|
| DC TT Geared Motor | 2        | Rear wheel drive (left and right motors)    |
| SG90 Servo Motor   | 1        | Front wheel steering mechanism              |
| 5V DC Cooling Fan  | 1        | Raspberry Pi heat management                |

---

## 💡 Indicators (LEDs)

| LED Type          | Quantity | Functionality                          |
|-------------------|----------|----------------------------------------|
| Red LED           | 4        | Brake lights & Blind Spot Warning      |
| Yellow LED        | 4        | Hazard indicators                      |
| White LED         | 4        | Adaptive headlights (ALS system)       |

---

## 🔌 Power & Supply

| Component            | Quantity | Description                                     |
|----------------------|----------|-------------------------------------------------|
| 18650 3.7V Battery   | 2        | Power supply for motors and ESP32               |
| 5V 3A Power Bank     | 1        | Power supply for Raspberry Pi                   |

---

## 🔧 Electronic Modules

| Module               | Quantity | Description                            |
|----------------------|----------|----------------------------------------|
| L298N Motor Driver   | 1        | Controls DC motors (rear wheel drive)  |

---

## 🔗 Communication

| Interface        | Use Case                               |
|------------------|----------------------------------------|
| I2C              | Communication between ESP32 and RPi    |
| UART (Bluetooth) | Manual control via mobile app          |

---

## 🛠️ Software & Development Tools

| Tool/Library           | Platform      | Purpose                             |
|------------------------|---------------|-------------------------------------|
| Arduino IDE            | ESP32         | Programming & deployment            |
| FreeRTOS               | ESP32         | Real-time task scheduling           |
| Python + OpenCV        | Raspberry Pi  | Image processing (TSDR)             |
| YOLOv8                 | Raspberry Pi  | Traffic sign detection & recognition|

---

> 📝 *All items were selected based on low cost, availability, and ease of integration for prototyping purposes.*

