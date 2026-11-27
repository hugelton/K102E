# K102E

Firmware and PCB Gerber files for the Hügelton Instruments K102E Digital Compound Oscillator (RP2040).

## Firmware

Source: `K102E_firmware/`

### Build

1. Install the Arduino IDE and the RP2040 board package
2. Install the libraries: FastLED, I2S, MCP_ADC
3. Open `K102E_firmware/K102E_firmware.ino`
4. Set the CPU speed to **133MHz** (150MHz or higher freezes the module)
5. Set Tools → Optimize to **Standard (-O)** (other levels cause stack overflow)
6. Upload via USB or SWD

Prebuilt updates: https://updater.hugelton.com

## Hardware

Gerber files for the PCBs are in `hardware/`:

- `K102E_Main_gerber.zip` - Main board
- `K102E_SwitchBoard_1.0_gerber.zip` - SwitchBoard

The front panel is not included.

## License

Copyright (c) 2024-2025 Hügelton Instruments, Leo Kuroshita

All files in this repository are licensed under the Hügelton Public License 1.0
(`SPDX-License-Identifier: LicenseRef-HPL-1.0`). See [LICENSE](LICENSE).
