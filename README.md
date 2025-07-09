# DualSense Controller Interface for Raspberry Pi Pico 2W

This project enables Bluetooth connectivity between a PlayStation 5 DualSense controller and a Raspberry Pi Pico 2W microcontroller. The interface provides real-time access to controller inputs including buttons, joysticks, triggers, and D-pad.

## Features

- Bluetooth HID Host implementation for DualSense controller
- Real-time state monitoring of all controller inputs
- Joystick axis normalization with configurable deadzone
- Optional polling frequency counter
- Simple API for accessing controller state
- Visual feedback using Pico's onboard LED in sample main.c file

## Hardware Requirements

1. Raspberry Pi Pico 2W
2. PlayStation 5 DualSense controller
3. USB cable for programming the Pico

## Software Requirements
- Visual Studio Code
- Raspberry Pi Pico extention for VSCode
- Raspberry Pi Pico SDK (version 2.1.1 was used for this project) [configure in VSCode extention GUI]
- Build tools (ARM/RISCV toolchain version 14.2Rel1) [configure in VSCode extention GUI]

## Setup Instructions

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/pico-dualsense-interface.git
cd pico-dualsense-interface
```
