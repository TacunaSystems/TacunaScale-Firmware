# Penner Bathing Scale Firmware

ESP32-S3 firmware for the Penner Bathing Scale, built with [PlatformIO](https://platformio.org/).

## Build Environment

- **Board:** ESP32-S3-DevKitC-1
- **Framework:** Arduino (Core 3.x via [pioarduino](https://github.com/pioarduino/platform-espressif32))
- **IDE:** VS Code + PlatformIO extension

## Getting Started

1. Install [VS Code](https://code.visualstudio.com/) and the [PlatformIO IDE extension](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide).
2. Clone this repo and open it in VS Code.
3. PlatformIO will automatically install the platform, toolchain, and library dependencies.
4. Click the PlatformIO **Build** button (checkmark icon) or run:
   ```
   pio run
   ```

## Flashing

### Via PlatformIO

Connect the board via USB and run:
```
pio run --target upload
```

### Via Web Flasher

1. Connect board to PC.
2. The new device should show up under ports in Device Manager:

    ![image](https://github.com/user-attachments/assets/76dcb878-7901-40a3-a7db-b9b6b6a256d2)
3. Visit: https://espressif.github.io/esptool-js/
4. Click 'Connect'
5. Select the COM port from step 2 and click 'Connect.'
6. Enter 0 in 'Flash Address' and click 'Choose File' and select the firmware.bin file.
7. Click 'Program'
8. Wait until the output shows 'Hash of data verified. Leaving...'
9. Power cycle the board and the new firmware should be loaded.

## Project Structure

```
├── platformio.ini          # PlatformIO project configuration
├── src/                    # Application source code
│   └── PennerScale.cpp     #   Main firmware (FreeRTOS tasks, UI, calibration)
├── lib/                    # Local/modified libraries
│   └── PRDC_AD7193/        #   AD7192 ADC driver (uses AD7193 library, see note below)
├── include/                # Project header files
├── test/                   # Unit tests
├── docs/                   # Reference documents
│   └── AD7192.xlsx         #   ADC filter word to settling time/Hz lookup table
├── archive/                # Archived Arduino IDE project (pre-PlatformIO)
└── .vscode/                # VS Code / PlatformIO IDE settings
```

### Firmware History

This firmware originated as an Arduino IDE sketch and has been tested on hundreds of units
internally. It was ported to PlatformIO for improved tooling and maintainability — the
application logic is unchanged from the proven Arduino version (archived in `archive/`).

### ADC Library Note

The hardware uses an **AD7192** but the firmware currently uses the third-party `PRDC_AD7193`
library (LGPL-3.0). The AD7192 and AD7193 share the same register map, so the library works
correctly — however the library's `begin()` ID check will return false due to the device ID
mismatch. This is expected and the return value is intentionally ignored. A proprietary
AD719x family library is planned to replace this dependency
(see [#3](https://github.com/TacunaSystems/Penner-Scale-Firmware/issues/3)).

### Third-Party Libraries

Before production release, all third-party library licenses must be reviewed for compliance:

| Library | License | Source |
|---------|---------|--------|
| PRDC_AD7193 | LGPL-3.0 | `lib/PRDC_AD7193/` (local, to be replaced) |
| U8g2 | BSD-2-Clause | PlatformIO registry |
| RunningAverage | MIT | PlatformIO registry |
