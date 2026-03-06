# SCPI Command Reference

Penner Bathing Scale — UART interface at 115200 baud, 8N1.

Commands follow the SCPI-2 (Standard Commands for Programmable Instruments) protocol.
Terminate each command with a newline (`\n`). Queries end with `?`.

## IEEE 488.2 Common Commands

| Command | Description |
|---------|-------------|
| `*CLS` | Clear status registers |
| `*ESE <val>` | Set event status enable register |
| `*ESE?` | Query event status enable register |
| `*ESR?` | Query event status register |
| `*IDN?` | Query instrument identification (Penner,BathingScale,serial,fw_ver) |
| `*OPC` | Set operation complete bit when all pending operations finish |
| `*OPC?` | Query operation complete (returns 1 when ready) |
| `*RST` | Reset — clears tare and running average |
| `*SRE <val>` | Set service request enable register |
| `*SRE?` | Query service request enable register |
| `*STB?` | Query status byte |
| `*TST?` | Self-test (returns 0 = pass) |
| `*WAI` | Wait for all pending operations to complete |

## Required SCPI Commands

| Command | Description |
|---------|-------------|
| `SYSTem:ERRor?` | Read and clear oldest error from queue |
| `SYSTem:ERRor:COUNt?` | Number of errors in queue |
| `SYSTem:VERSion?` | SCPI version (1999.0) |
| `STATus:QUEStionable?` | Query questionable status event register |
| `STATus:QUEStionable:ENABle <val>` | Set questionable status enable mask |
| `STATus:QUEStionable:ENABle?` | Query questionable status enable mask |
| `STATus:PRESet` | Reset status registers to default |

## Measurement Commands

| Command | Description | Returns |
|---------|-------------|---------|
| `MEASure:WEIGht?` | Averaged weight in current display unit | Float (kg or lb) |
| `MEASure:WEIGht:RAW?` | Combined raw ADC counts (Ch0 + Ch1 or Ch0 only) | Int32 |
| `MEASure:WEIGht:RAW:CH0?` | Raw ADC counts, channel 0 | Int32 |
| `MEASure:WEIGht:RAW:CH1?` | Raw ADC counts, channel 1 | Int32 |
| `MEASure:WEIGht:MAX?` | Peak weight recorded since last reset | Float |
| `MEASure:WEIGht:MAX <val>` | Set/reset peak weight tracker (persists to EEPROM) | — |
| `MEASure:WEIGht:AVERage:COUNt?` | Samples currently in running average buffer | Int |
| `MEASure:WEIGht:AVERage:SIZE?` | Running average buffer size (compile-time constant) | Int |

## Configuration Commands

| Command | Description | Parameter |
|---------|-------------|-----------|
| `CONFigure:UNIT <KG\|LB>` | Set display unit | `KG` or `LB` |
| `CONFigure:UNIT?` | Query current display unit | — |
| `CONFigure:TARE` | Tare — subtract current weight from future readings | — |
| `CONFigure:TARE?` | Query current tare offset | — |
| `CONFigure:ZERO` | Set current ADC reading as zero reference (persists to EEPROM) | — |
| `CONFigure:ADC:RATE <1-1023>` | Set AD7193 filter output rate (runtime only) | Int |
| `CONFigure:ADC:RATE?` | Query current ADC rate setting | — |
| `CONFigure:ADC:FILTer <SINC3\|SINC4>` | Set digital filter type (runtime only) | `SINC3` or `SINC4` |
| `CONFigure:ADC:FILTer?` | Query current filter type | — |
| `CONFigure:ADC:NOTCh <ON\|OFF>` | Enable/disable 50/60 Hz notch rejection (runtime only) | Boolean |
| `CONFigure:ADC:NOTCh?` | Query notch filter state | — |

## Calibration Commands

All set commands persist immediately to EEPROM.

| Command | Description | Parameter |
|---------|-------------|-----------|
| `CALibration:VALue?` | Query calibration factor (ADC counts per unit weight) | — |
| `CALibration:VALue <val>` | Set calibration factor (must be > 0) | Float |
| `CALibration:ZERO?` | Query zero-reference ADC value | — |
| `CALibration:ZERO <val>` | Set zero-reference ADC value directly | Int32 |
| `CALibration:WEIGht?` | Query calibration weight | — |
| `CALibration:WEIGht <val>` | Set calibration weight | UInt32 |
| `CALibration:UNIT?` | Query calibration unit | — |
| `CALibration:UNIT <KG\|LB>` | Set calibration unit | `KG` or `LB` |
| `CALibration:ZERO:EXEC` | Block until settled (~2.5s), capture zero reference, persist | — |
| `CALibration:SPAN:EXEC` | Block until settled (~2.5s), auto-compute calValue, persist | — |

## System Commands

| Command | Description | Parameter |
|---------|-------------|-----------|
| `SYSTem:BACKlight <ON\|OFF>` | Turn LCD backlight on/off | Boolean |
| `SYSTem:BACKlight?` | Query backlight state | — |
| `SYSTem:BACKlight:PWM <0-100>` | Set backlight PWM duty cycle (percent, persists to EEPROM) | Int (0-100) |
| `SYSTem:BACKlight:PWM?` | Query backlight PWM duty cycle (percent) | — |
| `SYSTem:POWer:VOLTage:BATTery?` | Battery input voltage | — |
| `SYSTem:POWer:VOLTage:SUPPly?` | 5V rail voltage | — |
| `SYSTem:POWer:DOWN` | Trigger power-down sequence | — |
| `SYSTem:ECHO <ON\|OFF>` | Enable/disable command echo (persists to EEPROM) | Boolean |
| `SYSTem:ECHO?` | Query echo state | — |
| `SYSTem:PROMpt <ON\|OFF>` | Enable/disable prompt after responses (persists to EEPROM) | Boolean |
| `SYSTem:PROMpt?` | Query prompt state | — |
| `SYSTem:EEPROM?` | Dump all EEPROM values (comma-separated key=value pairs) | — |
| `SYSTem:EEPROM:COMMit` | Flush pending EEPROM writes to flash | — |

## Diagnostic Commands

Available when `FREERTOS_DIAG` is enabled in `appconfig.h` (default: off).

| Command | Description |
|---------|-------------|
| `SYSTem:DIAGnostic:STATS?` | FreeRTOS CPU run-time percentage per task |
| `SYSTem:DIAGnostic:LIST?` | FreeRTOS task state, priority, and stack watermark |

## Debug Log Commands

Available when `SCPI_DEBUG` is enabled in `appconfig.h` (default: on).

| Command | Description |
|---------|-------------|
| `SYSTem:LOG?` | Read debug log from RAM ring buffer (2 KB) |
| `SYSTem:LOG:CLEar` | Clear the debug log |

## EEPROM Map

All calibration values persist across power cycles. The `SYSTem:EEPROM?` command
reads all fields directly from flash.

| Address | Field | Type | Persisted by |
|---------|-------|------|-------------|
| 0 | calValue | float | `CAL:VAL`, `CAL:SPAN:EXEC`, calibration routine |
| 4 | zeroValue | int32_t | `CAL:ZERO`, `CAL:ZERO:EXEC`, `CAL:SPAN:EXEC`, `CONF:ZERO`, calibration routine |
| 8 | backlightEnable | enum (int) | Power-down |
| 12 | unitVal | enum (int) | Power-down |
| 16 | calWeight | uint32_t | `CAL:WEIG`, calibration routine |
| 20 | calUnit | enum (int) | `CAL:UNIT`, calibration routine |
| 24 | extADCweightMax | float | `MEAS:WEIG:MAX`, power-down |
| 28 | backlightPWM | uint8_t | `SYST:BACK:PWM`, power-down |
| 29 | echo | uint8_t | `SYST:ECHO`, power-down |
| 30 | prompt | uint8_t | `SYST:PROM`, power-down |

## Examples

```
# Identify the instrument
*IDN?
→ Penner,BathingScale,00000000,1.0.0

# Read weight
MEAS:WEIG?
→ 25.4321

# Tare the scale
CONF:TARE

# Switch to pounds
CONF:UNIT LB

# Read all EEPROM values
SYST:EEPROM?
→ calValue=7168.220215,zeroValue=8295856,backlight=1,unit=KG,calWeight=100,calUnit=KG,weightMax=25.4321,backlightPWM=31,echo=1,prompt=1

# Reset peak weight
MEAS:WEIG:MAX 0

# Check for errors
SYST:ERR?
→ 0,"No error"

# Remote calibration workflow
CAL:UNIT KG
CAL:WEIG 100
# (unload scale)
CAL:ZERO:EXEC
→ (blocks ~2.5s, captures zero)
# (load 100 kg reference weight)
CAL:SPAN:EXEC
→ (blocks ~2.5s, computes calValue)
MEAS:WEIG?
→ 100.0000

# ADC tuning
CONF:ADC:RATE 300
CONF:ADC:RATE?
→ 300
CONF:ADC:FILT SINC3
CONF:ADC:FILT?
→ SINC3
```
