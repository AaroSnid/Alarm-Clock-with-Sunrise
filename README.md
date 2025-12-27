# Alarm Clock with Sunrise Simulator

This project is an STM32 powered alarm clock with an array of blue LEDs to gradually wake the user in the morning simulating a sunrise leading to an easier wake up experience. This device also features a menu system navigatable using pushbuttons to control LED max brightness, alarm time, noise level, and time display mode (12hr or 24hr clock). The system interface is a 4 digit 7 segment display interfaced with a shift register IC to minimise GPIO usage which also serves to display the time during regular operation.

### Hardware
The project hardware is:
- STM32 NUCLEO-F401RE Microcontroller and Debugger
- 74HC595 Shift Register
- CL3641BH Common anode 4 digit 7 Segment display

### STM32 Peripherals
This project features the usage of many STM32 peripherals, including:
- Timer channel PWM generation for the brightness of the LEDs, and register modification for gradual change
- Timer RTC with matched clock speed to accurately track the passage of time
- System interrupts to handle the user button input with software debouncing
- SPI interface for controlling the shift register at high speeds with minimal GPIO and processing power

At the moment, this repository only includes is the main.cpp file. The rest of the STM32 project files will be added soon.
