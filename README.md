# ST-T42 Closed-Loop Stepper Drive

ST-T42 is a back-mounted closed-loop controller for 42 mm two-phase stepper
motors. The current design uses an STM32G431, TMC2209 and MT6701, with UART,
Classic CAN and PWM control interfaces.

## Repository layout

```text
hardware/
  ProPrj_ST-T42 V1.2_2026-08-20.epro2  JLCEDA/EasyEDA Pro hardware project
  LICENSE                              CERN-OHL-S-2.0
firmware/
  STM32G431CBT6_test/                  STM32CubeMX and Keil firmware source
  SDK/                                 STM32 HAL UART/CAN control SDK
  release/ST-T42_V1.2.hex              Ready-to-flash firmware image
  LICENSE                              GPL-3.0-only
```

## Build

Open
`firmware/STM32G431CBT6_test/MDK-ARM/STM32G431CBT6_test.uvprojx` in Keil
MDK-ARM, or regenerate the project from
`firmware/STM32G431CBT6_test/STM32G431CBT6_test.ioc` with STM32CubeMX.

Build products and IDE-local files are ignored. Only reviewed release HEX
files under `firmware/release/` are committed.

## Licensing

- Hardware design source under `hardware/` is licensed under
  CERN-OHL-S-2.0.
- Original firmware and SDK code under `firmware/` is licensed under
  GPL-3.0-only.
- STM32 HAL, CMSIS, startup code and other third-party files retain the
  licenses and notices shipped in their respective directories.

See the README and LICENSE file in each top-level directory for details.
