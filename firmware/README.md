# Active Tracker Firmware

This repository contains the firmware for the Active Tracker device, a low-power animal tracking system built on the Raspberry Pi Pico platform.

## Overview

The Active Tracker is designed for wildlife monitoring with an emphasis on power efficiency and reliable data collection. The firmware manages several key components:

- GPS location tracking
- IMU-based activity classification using TensorFlow Lite
- Low-power operation modes
- Flash storage for data logging
- LoRa communication (planned)

## Project Structure

```
firmware/
├── classifier/     # Activity classification using IMU data and TensorFlow Lite
├── gps/            # GPS module interface
├── include/        # Common header files
├── lib/            # Shared libraries (I2C bus, etc.)
├── lora/           # LoRa communication (in development)
├── lowpower/       # Power management
├── storage/        # Flash storage management
├── external/       # External dependencies
│   ├── pico-sdk/   # Raspberry Pi Pico SDK
│   ├── pico-extras/ # Additional Pico libraries
│   └── pico-tflmicro/ # TensorFlow Lite for Microcontrollers
└── main.cpp        # Main application entry point
```

## Building the Firmware

### Prerequisites

- CMake (3.12 or newer)
- Arm GNU Toolchain
- Raspberry Pi Pico SDK
- Python 3.x (for SDK scripts)

### Setup

1. Clone this repository with submodules:
   ```
   git clone --recursive https://github.com/ecological-data-science/active-tracker.git
   ```

2. If you didn't clone with `--recursive`, initialize the submodules:
   ```
   git submodule update --init --recursive
   ```

   

### Compilation

1. Create a build directory and navigate to it:
   ```
   mkdir build
   cd build
   ```

2. Generate the build files with CMake:
   ```
   cmake -DPICO_BOARD=pimoroni_tiny2040 ..
   ```

3. Build the firmware:
   ```
   make
   ```

4. The compiled firmware will be available as `pico_tracker.uf2` in the build directory.

## Flashing the Firmware

1. Connect the Raspberry Pi Pico to your computer while holding the BOOTSEL button.
2. Release the button after connecting. The Pico should appear as a mass storage device.
3. Copy the `pico_tracker.uf2` file to the Pico drive.
4. The Pico will automatically reboot and run the new firmware.

## Hardware Connections

The firmware is configured to use the following default connections:

- I2C0: Used for GPS and IMU communication
  - SDA: GPIO4 (Pin 6)
  - SCL: GPIO5 (Pin 7)
- RGB LED indicators:
  - Red: GPIO18
  - Green: GPIO19
  - Blue: GPIO20

## Power Management

The firmware implements several power-saving strategies:

1. Sleep modes between sensor readings
2. Selective activation of peripherals
3. Duty cycling of power-hungry components like GPS

## Data Storage

Sensor data is stored in flash memory using a lightweight filesystem. The data format includes:

- Timestamp
- GPS coordinates
- Activity classification results


## Contact

colin.torney@glasgow.ac.uk

