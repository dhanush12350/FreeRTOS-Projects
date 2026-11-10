🚀 FreeRTOS Advanced Embedded Systems & Industrial Smart Grid Gateway
A collection of embedded firmware projects exploring FreeRTOS, real-time task scheduling, control systems, industrial communication, and microcontroller-based applications.

The repository targets microcontroller platforms including the Raspberry Pi Pico (RP2040), STM32, and ESP32. Each project focuses on practical embedded design concepts, from interrupt handling and synchronization to telemetry and closed-loop control.
📂 Project Portfolio
Project	Key Technical Features	Source Code
🤖 Dual Motor Robot Controller	RP2040, FreeRTOS, 50 Hz PID control, quadrature encoder feedback	[View Source](./Pico_Dual_Motor_FreeRTOS.c)
🛡️ Industrial Safety & Protection Monitor	Analog watchdog interrupt, CAN telemetry queue, fault monitoring	[View Source](./FreeRTOS_Industrial_Monitor.c)
⚙️ 12-Axis Industrial Servo Controller	Mutex-based bus protection, queues, priority inversion management	[View Source](./FreeRTOS%20%20Motor%20control.c)
🚥 4-Way Traffic Light Controller	Finite State Machine (FSM), emergency vehicle handling, interrupt-driven events	[View Source](./FreeRTOS_Traffic_Light.c)
🔋 EV Battery Management System	State of Charge (SoC) estimation, coulomb counting, 8-cell passive balancing	[View Source](./FreeRTOS_EV_BMS_Controller.c)
🛸 Autonomous UAV Flight Computer	Quaternion-based filtering, 1 kHz estimation loop, 400 Hz PID control	[View Source](./FreeRTOS_UAV_Flight_Controller.c)
⚡ Industrial Smart Grid Gateway	Modbus RTU, CANopen, lock-free ring buffer, EEPROM management, UART CLI	[View Source](./FreeRTOS_Industrial_Smart_Grid_Gateway.c)


⚡ Flagship Project: Industrial Smart Grid Telemetry Gateway
The Industrial Smart Grid Gateway is the flagship project in this repository. It combines real-time task scheduling, industrial communication protocols, telemetry processing, and fault management in a single firmware architecture.
Core Architecture
- 8 concurrent FreeRTOS tasks: Telemetry acquisition, power analysis, Modbus RTU, CANopen, fault management, EEPROM storage, CLI shell, and watchdog supervision.
- Industrial communication: Modbus RTU Master with CRC16 and CANopen TPDO/RPDO communication.
- Data handling: Lock-free ring buffer for ADC measurement streaming.
- Fault management: Dedicated fault monitoring and watchdog supervision.
- Diagnostics: Interactive UART-based Command Line Interface (CLI).
- Embedded storage: EEPROM-based configuration and data management.
Implementation note: The flagship source file is reported as 1,026 lines. See the linked source for the implementation details.

[**Explore the Smart Grid Gateway source code →**](./FreeRTOS_Industrial_Smart_Grid_Gateway.c)
🛠️ Development Environment & Build Setup
Target Platforms
- Raspberry Pi Pico (RP2040)
- STM32 microcontrollers
- ESP32 microcontrollers
Technologies & Concepts
- C programming and Embedded C
- FreeRTOS task scheduling and synchronization
- GPIO, ADC, timers, interrupts, and UART
- CAN and Modbus RTU communication
- PID control and finite state machines
- VS Code development environment
Repository Structure
FreeRTOS-Projects/
├── include/
│   └── Project header files
├── .vscode/
│   └── c_cpp_properties.json
├── Pico_Dual_Motor_FreeRTOS.c
├── FreeRTOS_Industrial_Monitor.c
├── FreeRTOS  Motor control.c
├── FreeRTOS_Traffic_Light.c
├── FreeRTOS_EV_BMS_Controller.c
├── FreeRTOS_UAV_Flight_Controller.c
└── FreeRTOS_Industrial_Smart_Grid_Gateway.c
VS Code IntelliSense configuration is provided through .vscode/c_cpp_properties.json.
🎯 Repository Focus
This portfolio demonstrates practical exploration of:
- Real-time embedded software architecture
- RTOS task management, priorities, and synchronization
- Interrupt-driven system design
- Motor control and feedback systems
- Industrial communication and telemetry
- Embedded diagnostics and fault handling
Author: Dhanush SJ
