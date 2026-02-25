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
│   └── PennerScale.ino
├── lib/                    # Local/modified libraries
│   └── PRDC_AD7193/        # Modified AD7193 ADC library
├── include/                # Project header files
├── test/                   # Unit tests
├── archive/                # Archived Arduino IDE project
└── .vscode/                # VS Code / PlatformIO IDE settings
```
