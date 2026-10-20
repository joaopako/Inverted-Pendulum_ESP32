# Inverted Pendulum ESP32
This project is an implementation of an inverted pendulum system using an ESP32 microcontroller. The inverted pendulum is a classic problem in control theory and robotics, where the goal is to balance a pendulum in an upright position on a moving cart.


## Components
For this project, the following hardware components are used:
- DRV8825 motor driver
- NEMA 17 stepper motor
- ESP32
- 5.1k ohms resistors (for current sensing / encoder pull-ups)
- 100uF capacitor (for filtering)

## Hardware Setup

| Source Component | Pin / Connection | Destination Component | Pin / Connection | Function / Notes |
| :--- | :--- | :--- | :--- | :--- |
| **ESP32** | GPIO 25 | **DRV8825** | STEP | Motor step control |
| **ESP32** | GPIO 26 | **DRV8825** | DIR | Motor direction control |
| **ESP32** | GPIO 27 | **DRV8825** | ENABLE | Enables/disables the driver (Active LOW) |
| **ESP32** | GPIO 14 | **DRV8825** | M0 (MS1) | Microstepping configuration |
| **ESP32** | GPIO 12 | **DRV8825** | M1 (MS2) | Microstepping configuration |
| **ESP32** | GPIO 13 | **DRV8825** | M2 (MS3) | Microstepping configuration |
| **ESP32** | GPIO 32 | **Encoder** | Channel A (CLK) | Pendulum position reading |
| **ESP32** | GPIO 33 | **Encoder** | Channel B (DT) | Pendulum position reading |
| **ESP32** | 3.3V / GND | **Encoder** | VCC / GND | Pendulum sensor power supply |
| **DRV8825** | RESET / SLEEP | **DRV8825** | RESET <-> SLEEP | Connect both pins together (Jumper wire) |
| **DRV8825** | GND / FAULT | **ESP32** | GND | Common logic ground |
| **DRV8825** | VMOT / GND | **External Power** | V+ / V- (12V-24V) | Motor power supply |
| **100µF Capacitor**| Polarity +/- | **DRV8825** | VMOT / GND | **Filter** (Mandatory across power pins) |
| **DRV8825** | A1, A2, B1, B2 | **NEMA 17** | Coils A and B | Physical connection to motor phases |

> ⚠️ **Critical Circuit Notes:**
> 1. **100µF Capacitor:** This must be placed as close as possible to the **VMOT** and **GND** pins on the DRV8825 board to prevent voltage spikes from destroying the driver.
> 2. **RESET and SLEEP Pins:** The DRV8825 requires a HIGH logic signal on the `RESET` pin to operate. The easiest way to achieve this is by placing a physical jumper wire directly between the `RESET` and `SLEEP` pins.
> 3. **5.1k Ohm Resistors:** For an ESP32 and an encoder setup, these are typically used as **pull-up resistors** connected from `PIN_ENC_A` and `PIN_ENC_B` to the 3.3V rail. This prevents floating inputs and guarantees clean pulse readings from the pendulum.

## Web Interface
The project includes a web interface that allows users to monitor and control the inverted pendulum system remotely. The web interface provides real-time data visualization, control inputs, and system status updates. The ESP32 works as an Access Point (AP) and hosts a web server that serves the interface to connected clients.
It contains the parameters of the PID controller, the setpoint, and the current angle of the pendulum. The web interface is built using HTML, CSS, and JavaScript, and it communicates with the ESP32 using WebSockets for real-time updates.  
