# DualSense Controller Interface for Raspberry Pi Pico 2W

This project enables Bluetooth HID host functionality on the Raspberry Pi Pico 2W, allowing it to connect to a PlayStation 5 DualSense controller and read real-time inputs through an easy-to-use C API.

---

## Features

* Bluetooth HID Host implementation for DualSense controller
* Real-time monitoring of all controller inputs (buttons, joysticks, triggers, D-pad)
* Joystick axis normalization with configurable deadzone
* Normalized trigger (L2/R2) axis values (0–100)
* D-pad directional values (0–8) and button queries (square, cross, circle, triangle)
* Optional polling-frequency counter for performance diagnostics
* Simple API functions exposed in `DS5_hid_host.h`
* Sample `main.c` demonstrating usage and onboard LED feedback

---

## Hardware Requirements

1. Raspberry Pi Pico 2W module
2. PlayStation 5 DualSense controller
3. Micro‑USB cable for programming and power (or VBUS/VSYS pins)

---

## Software Requirements

* Visual Studio Code (or another C/C++ IDE)
* Raspberry Pi Pico SDK (version ≥ 2.1.1)
* ARM/RISC‑V toolchain (GCC 14.2Rel1 or compatible)
* CMake (version ≥ 3.13)
* `cmake`, `make`, and `git` available on your PATH

---

## Repository Structure

```text
Raspberry-pi-pico-2W-DS5-host/
├── btstack_config.h       # BTstack settings
├── CMakeLists.txt         # Build configuration
├── DS5h_hid_host.c        # HID host implementation and API
├── DS5_hid_host.h         # Public header with API declarations
├── main.c                 # Example usage demonstrating the API
├── pico_sdk_import.cmake  # Generated project file
└── README.md              # This documentation file
```

---

## Setup Instructions

### 1. Clone the Repository

```bash
git clone https://github.com/Pim-Hartendorp/Raspberry-pi-pico-2W-DS5-host.git
cd Raspberry-pi-pico-2W-DS5-host
```

### 2. Initialize the Pico SDK

This is usually handled by the Pico extension for VS Code. If not:

* If the Pico SDK is a Git submodule, run:

  ```bash
  git submodule update --init --recursive
  ```

git submodule update --init --recursive

````
- Otherwise, set the `PICO_SDK_PATH` environment variable:
  ```bash
export PICO_SDK_PATH=/path/to/pico-sdk
````

### 3. Build the Project

The Pico extension may auto-build CMake. if not, select a kit using this comand in the VSCode search bar: 
**> CMake: Select a Kit** after this press ctrl + shift + B to build the project.

Otherwise:

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

The output `.uf2` file will be in the `build/` directory.

### 4. Flash to the Pico

1. Hold **BOOTSEL** on the Pico and connect via USB.
2. It appears as a mass-storage device.
3. Copy the generated `.uf2` file onto it.
4. The Pico reboots and runs the HID host firmware.
5. Reconnect to access console output over USB (if needed).

---

## Usage

1. Put the DualSense into pairing mode (hold **PS** + **Create** until the light bar blinks rapidly).
2. The Pico will auto-connect using the address passed to `DS5_init()` in `main.c`.
3. Call the API functions to read joystick, button, trigger, and D-pad states.

### Example (`main.c`)

```c
#include <stdio.h>
#include <stdbool.h>
#include "DS5_hid_host.h"

int main(void) {
    const char *remote_addr = "01:23:45:67:89:AB";  // Replace with your controller's MAC
    stdio_init_all();
    DS5_init(remote_addr, false);

    while (true) {
        int8_t lx = DS5_left_joystick_X_axis(5); // the deadzone is 5 percent in this case so the outputs values at 6
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

| Function                                        | Description                                  |
| ----------------------------------------------- | -------------------------------------------- |
| `int DS5_init(const char *addr, bool f);`       | Initialize HID host and connect to `addr`.   |
| `int8_t DS5_left_joystick_X_axis(uint8_t dz);`  | Left stick X (-100…100) with deadzone `dz`.  |
| `int8_t DS5_left_joystick_Y_axis(uint8_t dz);`  | Left stick Y (-100…100) with deadzone `dz`.  |
| `int8_t DS5_right_joystick_X_axis(uint8_t dz);` | Right stick X (-100…100) with deadzone `dz`. |
| `int8_t DS5_right_joystick_Y_axis(uint8_t dz);` | Right stick Y (-100…100) with deadzone `dz`. |
| `uint8_t DS5_Dpad_value(void);`                 | D-pad direction (0…8).                       |
| `bool DS5_square_button(void);`                 | Square button pressed?                       |
| `bool DS5_cross_button(void);`                  | Cross button pressed?                        |
| `bool DS5_circle_button(void);`                 | Circle button pressed?                       |
| `bool DS5_triangle_button(void);`               | Triangle button pressed?                     |
| `bool DS5_L1_button(void);`                     | L1 button pressed?                           |
| `bool DS5_R1_button(void);`                     | R1 button pressed?                           |
| `bool DS5_L2_button(void);`                     | L2 button pressed?                           |
| `bool DS5_R2_button(void);`                     | R2 button pressed?                           |
| `bool DS5_create_button(void);`                 | Create/Share button pressed?                 |
| `bool DS5_options_button(void);`                | Options/Menu button pressed?                 |
| `bool DS5_L3_button(void);`                     | L3 (stick press) pressed?                    |
| `bool DS5_R3_button(void);`                     | R3 (stick press) pressed?                    |
| `bool DS5_ps_button(void);`                     | PS/Home button pressed?                      |
| `bool DS5_touchpad_button(void);`               | Touchpad click pressed?                      |
| `uint8_t DS5_L2_axis(void);`                    | Normalized L2 axis (0…100).                  |
| `uint8_t DS5_R2_axis(void);`                    | Normalized R2 axis (0…100).                  |

---

## Troubleshooting

* **Controller not connecting**: Ensure the DualSense is in pairing mode and the MAC address is correct.
* **High latency or low frequency**: Enable the frequency counter by passing `true` to `DS5_init()` (observed \~780–800 Hz).
* **Build errors**: Verify `PICO_SDK_PATH` and toolchain versions.

---

## License

This code (and any modifications) is distributed under the following terms, as originally authored by BlueKitchen GmbH:

> **BlueKitchen Non‑Commercial BSD‑Style License**  
> (Full text in [LICENSE](./LICENSE))

You may redistribute and modify the source as long as you retain the copyright
notice, conditions, and disclaimer.  Any use must be for non‑commercial, personal benefit only.

---

## Acknowledgements

* **BlueKitchen GmbH**: HID host implementation adapted from the Pico Keyboard example.
* **Raspberry Pi Foundation**: Pico SDK and example code.
