TacunaScale Firmware v1.1.0c (Beta)
====================================
Date: 2026-03-09

WHAT'S NEW (since v1.1.0b)
--------------------------
- Added Newton (N), Newton-meter (Nm), and pound-foot (lbft) display/calibration units
  alongside existing kg and lb
- Added SCPI force aliases: MEASure:FORCe and CALibration:FORCe work identically
  to their WEIGht counterparts
- Redesigned LCD layout: weight value centered on its own line (full 128px width,
  up to 8 digits), unit displayed right-justified on a separate line below
- Battery indicator moved to top-right corner
- Unit abbreviations use SI/NIST middle-dot formatting on display
- Bug fixes: undefined behavior in unit abbreviation array, SCPI GROSS
  short-form notation per SCPI-99
- Added thread safety for unit changes (bounds-check + critical section)

FLASHING
--------
1. Open https://espressif.github.io/esptool-js/ in Chrome or Edge
2. Connect the TacunaScale via USB
3. Click "Connect", select the serial port
4. Set flash address to 0x0
5. Select firmware_merged.bin
6. Click "Program"
7. After flashing, press the reset button or power-cycle the device

HARDWARE
--------
- MCU: ESP32-S3-Mini-1-N4R2 (4 MB flash, 2 MB PSRAM)
- ADC: AD7193 (24-bit sigma-delta)
- Display: 128x64 OLED (SSD1306, I2C)

COMMUNICATION
-------------
- Interface: UART (USB-CDC)
- Baud: 115200, 8N1
- Protocol: SCPI-2 (see SCPI_Commands.md for full reference)
- Query firmware version: SYST:FW?
- Expected response: 1.1.0c

KNOWN LIMITATIONS
-----------------
- Cross-type unit changes (e.g. kg -> Nm) change the label only; no physical
  conversion is applied. Calibration should be done in the unit type matching
  your sensor.

PLEASE TEST
-----------
- [ ] Display shows weight on one line, unit right-justified below
- [ ] Cycle through all 5 units via button press
- [ ] SCPI: CONF:UNIT KG / LB / N / NM / LBFT -- verify query returns correct string
- [ ] SCPI: MEAS:FORC? returns same value as MEAS:WEIG?
- [ ] Verify unit conversions within mass (kg<->lb) and torque (Nm<->lbft)
- [ ] Calibration routine cycles all 5 units
