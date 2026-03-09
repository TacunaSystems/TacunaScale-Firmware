# TacunaScale Firmware

ESP32-S3 based scale readout device with SCPI interface and OLED display.

## Build

Use PlatformIO on the **Windows side** via PowerShell (not WSL):

```powershell
# Build
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -d 'C:\Users\User\Documents\GitHub\TacunaScale-Firmware'

# Merge binary for ESP web flasher (single bin at offset 0x0)
python "$env:USERPROFILE\.platformio\packages\tool-esptoolpy\esptool.py" --chip esp32s3 merge_bin -o firmware_merged.bin --flash_mode dio --flash_size 4MB 0x0 .pio\build\esp32-s3-devkitc-1\bootloader.bin 0x8000 .pio\build\esp32-s3-devkitc-1\partitions.bin 0xe000 "$env:USERPROFILE\.platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin" 0x10000 .pio\build\esp32-s3-devkitc-1\firmware.bin
```

From WSL bash, prefix commands with `powershell.exe -Command "..."`.

## Flashing

Firmware is flashed via [ESP Web Flasher](https://espressif.github.io/esptool-js/) (esptool browser). Flash `firmware_merged.bin` at offset `0x0`.

## Release Packaging

Beta releases are zipped as `TacunaScale_v<version>.zip` containing:
- `firmware_merged.bin` — merged flash image
- `README_v<version>.txt` — changelog and test checklist for colleague
- `docs/SCPI_Commands.md` — full SCPI command reference

## Version

`FW_VER` is defined in `src/appconfig.h`. Update it there plus `docs/SCPI_Commands.md` header and IDN example when bumping.

## Key Files

- `src/appconfig.h` — version, enums, EEPROM map, shared config
- `src/PennerScale.cpp` — main firmware (display, ADC, calibration, tasks)
- `src/scpi_interface.cpp` — SCPI command handlers
- `src/scpi_interface.h` — SCPI IDN fields, buffer config
- `docs/SCPI_Commands.md` — SCPI command reference
- `platformio.ini` — build config (ESP32-S3, 4MB flash, Arduino framework)
