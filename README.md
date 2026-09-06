# 🚀 FreeRTOS Advanced Embedded Systems & Industrial Smart Grid Gateway

Welcome to the **FreeRTOS Advanced Embedded Systems Repository**. This repository contains production-grade, highly optimized C firmware projects targeting real-time microcontroller architectures (Raspberry Pi Pico RP2040, STM32, ESP32).

---

## 📂 Project Showcase & Code Index

| Project Name | Primary Features | Source Code |
| :--- | :--- | :--- |
| 🤖 **Dual Motor Robot Controller** | RP2040 FreeRTOS, 50Hz PID Loop, Quadrature Encoders | [`Pico_Dual_Motor_FreeRTOS.c`](./Pico_Dual_Motor_FreeRTOS.c) |
| 🛡️ **Industrial Safety & Protection Monitor** | Sub-µs Analog Watchdog ISR, CAN Telemetry Queue | [`FreeRTOS_Industrial_Monitor.c`](./FreeRTOS_Industrial_Monitor.c) |
| ⚙️ **12-Axis Industrial Servo Controller** | Mutex Bus Locks, Non-blocking Queues, Priority Inversion Avoidance | [`FreeRTOS  Motor control.c`](./FreeRTOS%20%20Motor%20control.c) |
| 🚥 **4-Way Traffic Light System** | Preemptive Emergency Vehicle ISR Queue, FSM State Machine | [`FreeRTOS_Traffic_Light.c`](./FreeRTOS_Traffic_Light.c) |
| 🔋 **EV Battery Management System (BMS)** | Coulomb Counting SoC Estimation, 8-Cell Passive Balancing | [`FreeRTOS_EV_BMS_Controller.c`](./FreeRTOS_EV_BMS_Controller.c) |
| 🛸 **Autonomous UAV Flight Computer** | 1kHz Quaternion Filter, 400Hz Rate PID Mixer | [`FreeRTOS_UAV_Flight_Controller.c`](./FreeRTOS_UAV_Flight_Controller.c) |
| ⚡ **Industrial Smart Grid Gateway (Flagship)** | **1,026 Lines**: Modbus RTU, CANopen TPDO, Lock-Free Ring Buffer, EEPROM CLI | [`FreeRTOS_Industrial_Smart_Grid_Gateway.c`](./FreeRTOS_Industrial_Smart_Grid_Gateway.c) |

---

## ⚡ Flagship Project: Industrial Smart Grid Telemetry Gateway

`FreeRTOS_Industrial_Smart_Grid_Gateway.c` is a 1,000+ line production-grade firmware suite featuring:
* **8 Concurrent Real-Time Tasks**: Telemetry, Power Analysis, Modbus RTU, CANopen, Fault Management, EEPROM Storage, CLI Shell, & Watchdog.
* **Protocol Engines**: Hardware CRC16 Modbus RTU Master & CANopen TPDO/RPDO.
* **High Throughput**: Lock-Free Ring Buffer for streaming ADC measurements without mutex contention.
* **Diagnostics**: Real-time interactive UART Command Line Interface (CLI).

---

## 📅 LinkedIn 3-Day Content Release Schedule

| Day | Topic / Video | Code Permalink |
| :--- | :--- | :--- |
| **Day 1** | 🤖 Dual Motor Controller (`dual motor`) | [View Code](https://github.com/dhanush12350/FreeRTOS-Projects/blob/main/Pico_Dual_Motor_FreeRTOS.c) |
| **Day 4** | 🛡️ Industrial Safety Monitor (`Industrial montoring`) | [View Code](https://github.com/dhanush12350/FreeRTOS-Projects/blob/main/FreeRTOS_Industrial_Monitor.c) |
| **Day 7** | ⚙️ 12-Axis Servo Controller (`RTOS motor`) | [View Code](https://github.com/dhanush12350/FreeRTOS-Projects/blob/main/FreeRTOS%20%20Motor%20control.c) |
| **Day 10** | 🚥 4-Way Traffic Light System (`Traffic RTOS`) | [View Code](https://github.com/dhanush12350/FreeRTOS-Projects/blob/main/FreeRTOS_Traffic_Light.c) |
| **Day 13** | 🔋 EV Battery Management System (BMS) | [View Code](https://github.com/dhanush12350/FreeRTOS-Projects/blob/main/FreeRTOS_EV_BMS_Controller.c) |
| **Day 16** | 🛸 UAV Flight Controller System | [View Code](https://github.com/dhanush12350/FreeRTOS-Projects/blob/main/FreeRTOS_UAV_Flight_Controller.c) |
| **Day 19** | ⚡ **FLAGSHIP: Smart Grid Gateway (1,000+ Lines)** | [View Code](https://github.com/dhanush12350/FreeRTOS-Projects/blob/main/FreeRTOS_Industrial_Smart_Grid_Gateway.c) |

---

## 🛠️ Build & VS Code Setup

All headers are organized in the [`include/`](./include) directory. 

VS Code IntelliSense is pre-configured in `.vscode/c_cpp_properties.json` for zero build/linter errors.
