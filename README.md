# DualSense Controller Interface for Raspberry Pi Pico 2W

This project enables Bluetooth HID host functionality on the Raspberry Pi Pico 2W, allowing it to connect to a PlayStation 5 DualSense controller and read real-time inputs through an easy-to-use C API.

---

## Features

* Bluetooth HID Host implementation for DualSense controller
* Real-time state monitoring of all controller inputs (buttons, joysticks, triggers, D-pad)
* Joystick axis normalization with configurable deadzone
* Normalized trigger (L2/R2) axis values (0–100)
* D-pad directional values (0–8) and button queries (square, cross, circle, triangle)
* Optional polling frequency counter for performance diagnostics
* Simple, intuitive API functions exposed in `DS5_hid_host.h`
* Sample `main.c` demonstrating usage and visual feedback via onboard LED

---

## Hardware Requirements

1. Raspberry Pi Pico 2W module
2. PlayStation 5 DualSense controller
3. Micro USB cable for programming and power

---

## Software Requirements

* Visual Studio Code (or another C/C++ IDE)
* Raspberry Pi Pico SDK (version ≥ 2.1.1)
* ARM/RISC‑V toolchain (GCC 14.2Rel1 or compatible)
* CMake (version ≥ 3.13)
* `cmake`, `make`, and `git` on your PATH

---

## Repository Structure

```text
Raspberry-pi-pico-2W-DS5-host/
├── CMakeLists.txt           # Top-level build configuration
├── DS5h_hid_host.c          # HID host implementation and API
├── DS5_hid_host.h           # Public header with API declarations
├── main.c                   # Example usage demonstrating API
└── README.md                # This documentation file
```

---

## Setup Instructions

### 1. Clone the Repository

```bash
git clone https://github.com/Pim-Hartendorp/Raspberry-pi-pico-2W-DS5-host.git
cd Raspberry-pi-pico-2W-DS5-host
```

### 2. Initialize Pico SDK

If the Pico SDK is added as a Git submodule, run:

```bash
git submodule update --init --recursive
```

Otherwise, set the `PICO_SDK_PATH` environment variable to your SDK location:

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
```

### 3. Build the Project

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

The build outputs a `.uf2` file in the `build/` directory.

### 4. Flash to Pico

1. Hold **BOOTSEL** on the Pico and connect via USB.
2. It appears as a mass-storage device.
3. Copy the `.uf2` file onto it.
4. The Pico reboots and starts the HID host firmware.

---

## Usage

1. Put the DualSense into pairing mode (hold **PS** + **Create** until the light bar blinks).
2. The Pico auto-connects using the address passed to `DS5_init()` in `main.c`.
3. Call API functions to read joystick, button, trigger, and D-pad states.

### Example (`main.c`)

```c
#include <stdio.h>
#include "DS5_hid_host.h"

int main(void) {
    const char *remote_addr = "58:10:31:2B:C8:71";
    stdio_init_all();
    DS5_init(remote_addr, false);

    while (true) {
        int8_t lx = DS5_left_joystick_X_axis(5);
        int8_t ly = DS5_left_joystick_Y_axis(5);
        bool cross = DS5_cross_button();
        printf("LX=%d, LY=%d, Cross=%d\n", lx, ly, cross);
        tight_loop_contents();
    }
    return 0;
}
```

---

## API Reference

| Function                                            | Description                                  |
| --------------------------------------------------- | -------------------------------------------- |
| `int DS5_init(const char *addr, bool enable_freq);` | Initialize HID host and connect to `addr`.   |
| `int8_t DS5_left_joystick_X_axis(uint8_t dz);`      | Left stick X (-100…100) with deadzone `dz`.  |
| `int8_t DS5_left_joystick_Y_axis(uint8_t dz);`      | Left stick Y (-100…100) with deadzone `dz`.  |
| `int8_t DS5_right_joystick_X_axis(uint8_t dz);`     | Right stick X (-100…100) with deadzone `dz`. |
| `int8_t DS5_right_joystick_Y_axis(uint8_t dz);`     | Right stick Y (-100…100) with deadzone `dz`. |
| `uint8_t DS5_Dpad_value(void);`                     | D-pad direction (0…8).                       |
| `bool DS5_square_button(void);`                     | Square button pressed?                       |
| `bool DS5_cross_button(void);`                      | Cross button pressed?                        |
| `bool DS5_circle_button(void);`                     | Circle button pressed?                       |
| `bool DS5_triangle_button(void);`                   | Triangle button pressed?                     |
| `bool DS5_L1_button(void);`                         | L1 button pressed?                           |
| `bool DS5_R1_button(void);`                         | R1 button pressed?                           |
| `bool DS5_L2_button(void);`                         | L2 button pressed?                           |
| `bool DS5_R2_button(void);`                         | R2 button pressed?                           |
| `bool DS5_create_button(void);`                     | Create/Share button pressed?                 |
| `bool DS5_options_button(void);`                    | Options/Menu button pressed?                 |
| `bool DS5_L3_button(void);`                         | L3 (stick) pressed?                          |
| `bool DS5_R3_button(void);`                         | R3 (stick) pressed?                          |
| `bool DS5_ps_button(void);`                         | PS/Home button pressed?                      |
| `bool DS5_touchpad_button(void);`                   | Touchpad click pressed?                      |
| `uint8_t DS5_L2_axis(void);`                        | Normalized L2 axis (0…100).                  |
| `uint8_t DS5_R2_axis(void);`                        | Normalized R2 axis (0…100).                  |

---

## Troubleshooting

* **Controller not connecting**: Ensure pairing mode and correct MAC address.
* **Low polling frequency**: Call `DS5_init(addr, true)` to enable frequency printing.
* **Build errors**: Verify `PICO_SDK_PATH` and toolchain versions.

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

---

## Acknowledgements

* **BlueKitchen GmbH**: HID host implementation adapted from the Pico Keyboard example.
* **Raspberry Pi Foundation**: Pico SDK and example code.

---

