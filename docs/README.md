# ZETIN Drone Project

This repository contains code and documentation for the ZETIN Drone project, focusing on drone control, data analysis, and firmware development.

## Directory Structure

*   `docs/`: Project documentation, including this README, commit messages, and presentations.
*   `logs/`: Stores flight and drone operational logs in CSV format.
*   `scripts/`: Python scripts for drone control, analysis, and tuning.
*   `firmware/`: Contains the PlatformIO project for drone firmware development.
    *   `firmware/platformio_config/`: Configuration file for PlatformIO (`platformio.ini`).
    *   `firmware/src/`: Source code for the main drone firmware.
    *   `firmware/lib/`: Custom libraries used in the firmware.
    *   `firmware/include/`: Header files for the firmware.
    *   `firmware/examples/`: Various test and example firmware projects (e.g., DSHOT, PWM, IMU tests).
*   `test/`: Contains general test scripts, currently including `tcp_test.py`.

## Getting Started

### Firmware Development

The drone firmware is developed using PlatformIO. To get started with firmware development:

1.  Install [PlatformIO IDE](https://platformio.org/install).
2.  Open the `firmware/` directory as a PlatformIO project.
3.  Explore the main firmware in `firmware/src/` or various test examples in `firmware/examples/`.

### Drone Control & Analysis

The `scripts/` directory contains Python scripts for interacting with and analyzing drone data.

1.  Navigate to the `scripts/` directory.
2.  Run the desired Python script (e.g., `python Drone_Control_Dualsense.py` for control, `python Drone_Analasys.py` for analysis).

### Logs

Flight and drone logs are automatically saved in the `logs/` directory. These can be used for post-flight analysis and tuning.