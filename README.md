# Inverted Pendulum ESP32
This project is an implementation of an inverted pendulum system using an ESP32 microcontroller. The inverted pendulum is a classic problem in control theory and robotics, where the goal is to balance a pendulum in an upright position on a moving cart.

## Hardware
For this project, the following hardware components are used:
- DRV8833 motor driver
- NEMA 17 stepper motor
- ESP32
- 5.1k ohms resistors (for current sensing)
- 100uF capacitor (for filtering)

## Web Interface
The project includes a web interface that allows users to monitor and control the inverted pendulum system remotely. The web interface provides real-time data visualization, control inputs, and system status updates. The ESP32 works as an Access Point (AP) and hosts a web server that serves the interface to connected clients.
It contains the parameters of the PID controller, the setpoint, and the current angle of the pendulum. The web interface is built using HTML, CSS, and JavaScript, and it communicates with the ESP32 using WebSockets for real-time updates.  
