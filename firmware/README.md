# ST-T42 Firmware

## Contents

- `STM32G431CBT6_test/`: STM32G431 application source and Keil/CubeMX project.
- `SDK/`: transport-independent Step42 protocol plus STM32 HAL UART and CAN
  adapters.
- `release/`: reviewed firmware images intended for end users.
- `tools/upper-computer/`: packaged Windows upper-computer application.

The STM32 project was generated with STM32CubeMX 6.15.0 and STM32CubeG4
V1.6.2. Only the HAL and CMSIS files required to build this target are kept in
the repository; unused CMSIS DSP, NN, RTOS, documentation, and template files
are intentionally omitted.

## License notice

Copyright (c) 2026 ST-T42 Project Contributors.

The original ST-T42 firmware and SDK code are licensed under the GNU General
Public License version 3 only (`GPL-3.0-only`). The complete license text is
provided in `LICENSE`.

Third-party components, including STM32 HAL and CMSIS files, retain their own
license notices. The GPL declaration does not replace those notices.
