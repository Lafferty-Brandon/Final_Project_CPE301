# Final_Project_CPE301
CPE301 Fall 2025 Final Project

Dr. Bashira Akter Anima


Section 1001
Solo Group:
1. Brandon Lafferty

This project implements an evaporative (swamp) cooler system using an Arduino Mega 2560. The system monitors ambient temperature and humidity using a DHT11 sensor and automatically controls a fan motor when the temperature exceeds a defined threshold. A water level sensor ensures safe operation by detecting low water conditions and transitioning the system into an error state when necessary.

The system is designed as a finite state machine with four operating states: Disabled, Idle, Running, and Error. User interaction is provided through dedicated Start, Stop, Reset, and vent control buttons. A stepper motor allows real-time adjustment of the vent direction, while a real-time clock (RTC) records timestamps for state transitions, sensor readings, fan activity, and vent movements. All events are logged to a host computer via UART.

Hardware Used:
1. Arduino AtMega 2560
2. DHT11 Temperature and Humidity Sensor
3. Water Level Sensor
4. DC Fan Motor and Blade
5. External Power Supply Board
6. 16x20 LCD Display
7. Stepper Motor
8. DS1307 Real-Time Clock Module
9. 5 Push Buttons
10. Yellow,Green,Red,Blue LEDs
11. 2N2222 Transistor
12. Diode Rectifier
13. Resistors (220 Ohm and 10k Ohm)

How to Run

1. Assemble the Hardware

  Connect all components according to the schematic, including the Arduino Mega 2560, DHT11 sensor, water level sensor, LCD, stepper motor, fan motor with external power supply, buttons, LEDs, and RTC module. Ensure the fan motor is powered through the separate power supply board and not directly from the Arduino.

2. Upload the Code

  Open the provided Arduino project file and upload it to the Arduino Mega 2560 using the Arduino IDE. No additional configuration is required beyond selecting the correct board and port.

3. Open Serial Monitor

  Open the Serial Monitor and set the baud rate to 9600 baud. This allows viewing of system logs, including timestamps, state transitions, sensor readings, and vent position updates.

4. Power the System

  Apply power to the Arduino and the external motor power supply. On startup, the system enters the DISABLED state and displays a disabled message on the LCD.

5. Start the System

  Press the Start button to enable the system. The system transitions to the IDLE state, begins monitoring temperature, humidity, and water level, and updates the LCD once per minute.

6. Automatic Operation

  If the temperature reaches or exceeds the threshold (24.3 °C), the system enters the RUNNING state and activates the fan.
  When the temperature drops below the threshold, the system returns to the IDLE state.

7. Vent Control

  Use the Vent Left and Vent Right buttons to adjust the vent direction. The stepper motor moves in fixed increments, and each adjustment is logged with a timestamp.

8. Water Level Error Handling

  If the water level drops below the threshold, the system transitions to the ERROR state, disables the fan, and displays a warning on the LCD.
Refill the reservoir and press the Reset button to return to the IDLE state.

9. Stop the System

  Press the Stop button at any time to disable the system and return to the DISABLED state.

Video Demonstration Link:
https://youtu.be/6TfMThOKhvw
