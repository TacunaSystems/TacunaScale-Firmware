# TacunaScale SCPI Command Reference

**Firmware:** 1.2.0a | **Updated:** 2026-03-10

## Overview

Firmware v1.2.0 introduces **dual-channel** support. The TacunaScale reads two
independent ADC channels (CH0 and CH1), each with its own calibration, unit,
tare, and filter settings. All measurement and most configuration/calibration
commands require a `:CH0` or `:CH1` channel suffix. Measurement commands also
provide `:SUM?` variants that combine both channels (CH1 is converted to CH0's
display unit before summing).

## Communication

| Parameter | Value |
|-----------|-------|
| Interface | UART (serial) |
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Line terminator | `\n` (newline) |

## Command Syntax

Commands follow the **SCPI-2** (Standard Commands for Programmable Instruments)
protocol. Queries end with `?` and return a value. Set commands have no response.

**Short-form notation:** uppercase letters are the required abbreviation,
lowercase letters may be omitted. For example, `CONFigure:UNIT:CH0` can be sent
as `CONF:UNIT:CH0`. Both forms are equivalent.

**Echo and prompt:** by default the scale echoes each received command and
prefixes responses with `> `. Both behaviors are configurable via
`SYSTem:ECHO` and `SYSTem:PROMpt`.

**Errors:** invalid commands push an error onto the SCPI error queue, readable
via `SYSTem:ERRor?`. The queue holds up to 17 entries.

## IEEE 488.2 Common Commands

| Command | Description |
|---------|-------------|
| `*CLS` | Clear status registers |
| `*ESE <val>` | Set event status enable register |
| `*ESE?` | Query event status enable register |
| `*ESR?` | Query event status register |
| `*IDN?` | Query instrument identification (Tacuna Systems,TacunaScale,serial,fw_ver) |
| `*OPC` | Set operation complete bit when all pending operations finish |
| `*OPC?` | Query operation complete (returns 1 when ready) |
| `*RST` | Reset — clears tare and running average for all channels |
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

All measurement commands are **per-channel**. Replace `<ch>` with `CH0` or
`CH1`. Where noted, a `:SUM?` variant is also available.

| Command | Description | Returns |
|---------|-------------|---------|
| `MEASure:WEIGht:<ch>?` | Averaged weight in channel's display unit | Float |
| `MEASure:WEIGht:SUM?` | CH0 + CH1 summed (CH1 converted to CH0's unit) | Float |
| `MEASure:WEIGht:RAW:<ch>?` | Raw ADC counts for channel | Int32 |
| `MEASure:WEIGht:RAW:SUM?` | CH0 + CH1 raw ADC counts summed | Int32 |
| `MEASure:WEIGht:MAX:<ch>?` | Peak weight recorded since last clear, in current display unit | Float |
| `MEASure:WEIGht:MAX:<ch> <val>` | Set/clear peak weight (e.g. `0` to reset; persists to EEPROM) | — |
| `MEASure:WEIGht:AVERage:COUNt:<ch>?` | Samples currently in running average buffer | Int |
| `MEASure:WEIGht:AVERage:SIZE:<ch>?` | Running average buffer size (compile-time, default 5) | Int |
| `MEASure:WEIGht:SDEViation:<ch>?` | Standard deviation of running average buffer | Float |
| `MEASure:WEIGht:STABle:<ch>?` | Stability flag: 1 = settled, 0 = unstable | Bool (0/1) |
| `MEASure:WEIGht:GROSs:<ch>?` | Weight before tare subtraction, in display unit | Float |
| `MEASure:WEIGht:OVERload:<ch>?` | Overload flag: 1 = abs(weight) > capacity, 0 = OK | Bool (0/1) |

All `MEASure:WEIGht` commands are also available as `MEASure:FORCe` aliases
(e.g. `MEAS:FORC:CH0?` is equivalent to `MEAS:WEIG:CH0?`).

**Peak weight note:** The peak value is always stored and reported in the
channel's current display unit. When the unit is changed via `CONF:UNIT`, the
peak value is automatically converted. The peak is persisted to EEPROM on
power-down and on unit change, so it survives reboot. Intended for overload
forensics — query `MEAS:WEIG:MAX:<ch>?` alongside `CONF:UNIT:<ch>?` to
determine the historical peak load.

## Configuration Commands

### Display Mode

| Command | Description | Parameter |
|---------|-------------|-----------|
| `CONFigure:DISPlay:MODE <SINGle\|DUAL\|SUM>` | Set display mode (persists to EEPROM) | See below |
| `CONFigure:DISPlay:MODE?` | Query current display mode | — |

| Mode | Display Layout |
|------|---------------|
| `SINGle` | Full-screen CH0 weight and unit (default) |
| `DUAL` | Split-screen — CH0 top half, CH1 bottom half with divider |
| `SUM` | Full-screen combined CH0 + CH1 weight (CH1 converted to CH0's unit) |

### Units (per-channel)

| Token | Unit | Type | Conversions |
|-------|------|------|-------------|
| `KG` | Kilogram | Mass | KG ↔ LB, KG ↔ N |
| `LB` | Pound | Mass | LB ↔ KG, LB ↔ N |
| `N` | Newton | Force | N ↔ KG, N ↔ LB |
| `NM` | Newton-meter | Torque | NM ↔ LBFT |
| `LBFT` | Pound-foot | Torque | LBFT ↔ NM |

Switching between compatible units auto-converts overload capacity and peak
weight. Cross-type changes (e.g. mass → torque) change only the label.
Each channel has an independent unit setting.

| Command | Description | Parameter |
|---------|-------------|-----------|
| `CONFigure:UNIT:<ch> <KG\|LB\|N\|NM\|LBFT>` | Set display unit for channel | Unit token |
| `CONFigure:UNIT:<ch>?` | Query current display unit for channel | — |
| `CONFigure:TARE:<ch> [<value>]` | Tare — no param: auto-tare from current reading. With param: set tare offset directly (in cal units) | Float (optional) |
| `CONFigure:TARE:<ch>?` | Query current tare offset for channel | — |
| `CONFigure:ZERO:<ch>` | Set current ADC reading as zero reference (persists to EEPROM) | — |

### ADC Settings

ADC rate, filter type, and notch are **shared** (one physical ADC).
ADC invert is **per-channel**.

| Command | Description | Parameter |
|---------|-------------|-----------|
| `CONFigure:ADC:RATE <1-1023>` | Set AD7193 filter output rate (runtime only) | Int |
| `CONFigure:ADC:RATE?` | Query current ADC rate setting | — |
| `CONFigure:ADC:FILTer <SINC3\|SINC4>` | Set digital filter type (runtime only) | `SINC3` or `SINC4` |
| `CONFigure:ADC:FILTer?` | Query current filter type | — |
| `CONFigure:ADC:NOTCh <ON\|OFF>` | Enable/disable 50/60 Hz notch rejection (runtime only) | Boolean |
| `CONFigure:ADC:NOTCh?` | Query notch filter state | — |
| `CONFigure:ADC:INVert:<ch> <ON\|OFF>` | Invert ADC polarity for channel (persists). Negates raw values, zero, tare, and peak — no re-calibration required. Use when load cells produce decreasing counts under load. | Boolean |
| `CONFigure:ADC:INVert:<ch>?` | Query ADC inversion state for channel | Bool (0/1) |

### Stability and Overload (per-channel)

| Command | Description | Parameter |
|---------|-------------|-----------|
| `CONFigure:STABility:THReshold:<ch> <val>` | Stability threshold as fraction of capacity (persists, default 0.0002 = 0.02%) | Float (> 0, ≤ 1) |
| `CONFigure:STABility:THReshold:<ch>?` | Query current stability threshold for channel | — |
| `CONFigure:OVERload:CAPacity:<ch> <val>` | Overload capacity in display units (persists, default 500). Auto-converts when display unit changes. | Float (> 0, ≤ 16777216) |
| `CONFigure:OVERload:CAPacity:<ch>?` | Query current overload capacity for channel | — |

### Adaptive Filter (per-channel)

| Command | Description | Parameter |
|---------|-------------|-----------|
| `CONFigure:FILTer:ADAPtive:<ch> <ON\|OFF>` | Enable/disable adaptive filter (persists, default ON) | Boolean |
| `CONFigure:FILTer:ADAPtive:<ch>?` | Query adaptive filter state for channel | — |
| `CONFigure:FILTer:ADAPtive:THReshold:<ch> <val>` | Deviation threshold as % of capacity (persists, default 1.0) | Float (> 0, ≤ 100) |
| `CONFigure:FILTer:ADAPtive:THReshold:<ch>?` | Query adaptive filter threshold for channel | — |
| `CONFigure:FILTer:ADAPtive:TIME:<ch> <val>` | Sustained deviation window in microseconds (persists, default 750000) | Int (1–60000000) |
| `CONFigure:FILTer:ADAPtive:TIME:<ch>?` | Query adaptive filter time window for channel | — |

The adaptive filter detects sustained directional weight changes (e.g. a new
load placed on the scale) and clears the running average for immediate display
response. Oscillatory motion (e.g. a patient shifting on a chair scale) resets
the detection timer, so normal averaging continues undisturbed.

Each new reading is compared to the current average. If the deviation exceeds
`threshold × capacity / 100` and persists in the same direction for longer than
the configured time window, the running average buffer is cleared. The time-based
window automatically scales with sample rate.

## Calibration Commands (per-channel)

All set commands persist immediately to EEPROM. Replace `<ch>` with `CH0` or
`CH1`.

| Command | Description | Parameter |
|---------|-------------|-----------|
| `CALibration:VALue:<ch>?` | Query calibration factor (ADC counts per unit weight) | — |
| `CALibration:VALue:<ch> <val>` | Set calibration factor (must be > 0) | Float |
| `CALibration:ZERO:<ch>?` | Query zero-reference ADC value | — |
| `CALibration:ZERO:<ch> <val>` | Set zero-reference ADC value directly (±16777215) | Int32 |
| `CALibration:WEIGht:<ch>?` | Query calibration weight | — |
| `CALibration:WEIGht:<ch> <val>` | Set calibration weight | UInt32 |
| `CALibration:UNIT:<ch>?` | Query calibration unit | — |
| `CALibration:UNIT:<ch> <KG\|LB\|N\|NM\|LBFT>` | Set calibration unit | Unit token |
| `CALibration:ZERO:EXEC:<ch>` | Block until settled (~2.5 s), capture zero reference, persist | — |
| `CALibration:SPAN:EXEC:<ch>` | Block until settled (~2.5 s), auto-compute calValue, persist | — |

**Negative span:** Span calibration accepts both positive and negative ADC
deltas (i.e. load cells where raw counts decrease under load). The calibration
factor (`calValue`) is always stored as a positive number. If the ADC delta is
negative, weight readings will show a negative sign until `CONF:ADC:INVert` is
enabled for that channel — no re-calibration is needed after toggling invert.

`CALibration:WEIGht` and `CALibration:WEIGht?` are also available as
`CALibration:FORCe` aliases.

## System Commands

| Command | Description | Parameter |
|---------|-------------|-----------|
| `SYSTem:BACKlight <ON\|OFF>` | Turn LCD backlight on/off | Boolean |
| `SYSTem:BACKlight?` | Query backlight state | — |
| `SYSTem:BACKlight:PWM <0-100>` | Set backlight PWM duty cycle (percent, persists) | Int (0–100) |
| `SYSTem:BACKlight:PWM?` | Query backlight PWM duty cycle (percent) | — |
| `SYSTem:POWer:VOLTage:BATTery?` | Battery input voltage | — |
| `SYSTem:POWer:VOLTage:SUPPly?` | 5 V rail voltage | — |
| `SYSTem:POWer:DOWN` | Trigger power-down sequence | — |
| `SYSTem:POWer:GOOD:VDD?` | 3.3 V rail power good signal | Bool (0/1) |
| `SYSTem:POWer:GOOD:V5A?` | 5 V rail power good signal | Bool (0/1) |
| `SYSTem:ECHO <ON\|OFF>` | Enable/disable command echo (persists) | Boolean |
| `SYSTem:ECHO?` | Query echo state | — |
| `SYSTem:PROMpt <ON\|OFF>` | Enable/disable response prompt prefix (persists) | Boolean |
| `SYSTem:PROMpt?` | Query prompt state | — |
| `SYSTem:CONFig:SW1?` | DIP switch 1 state (read-only, read at boot) | Bool (0/1) |
| `SYSTem:CONFig:SW2?` | DIP switch 2 state (read-only, read at boot) | Bool (0/1) |
| `SYSTem:FW?` | Firmware version string | String |
| `SYSTem:EEPROM?` | Dump all persisted EEPROM values (comma-separated key=value) | — |
| `SYSTem:EEPROM:COMMit` | Flush pending EEPROM writes to flash | — |
| `SYSTem:LOG?` | Read debug log from RAM ring buffer (2 KB). Requires `SCPI_DEBUG` (enabled by default). | — |
| `SYSTem:LOG:CLEar` | Clear the debug log. Requires `SCPI_DEBUG`. | — |
| `SYSTem:DIAGnostic:STATS?` | FreeRTOS task runtime stats. Requires `FREERTOS_DIAG` build flag. | — |
| `SYSTem:DIAGnostic:LIST?` | FreeRTOS task list. Requires `FREERTOS_DIAG` build flag. | — |

> **Note:** DIP switches (SW1, SW2) are read at boot but do not control any
> firmware feature in v1.2.0. They are reserved for future use. Display mode
> and all other settings are controlled via SCPI commands and persisted to EEPROM.

## EEPROM Map

All values persist across power cycles. The `SYSTem:EEPROM?` command reads all
fields directly from flash. CH0 and CH1 each have independent copies of
per-channel settings.

### Shared Settings

| Address | Field | Type | Persisted by |
|---------|-------|------|-------------|
| 0 | calValue[0] | float | `CAL:VAL:CH0`, `CAL:SPAN:EXEC:CH0`, calibration |
| 4 | zeroValue[0] | int32_t | `CAL:ZERO:CH0`, `CAL:ZERO:EXEC:CH0`, `CONF:ZERO:CH0`, calibration |
| 8 | backlightEnable | enum (int) | Power-down |
| 12 | unitVal[0] | enum (int) | `CONF:UNIT:CH0`, power-down |
| 16 | calWeight[0] | uint32_t | `CAL:WEIG:CH0`, calibration |
| 20 | calUnit[0] | enum (int) | `CAL:UNIT:CH0`, calibration |
| 24 | extADCweightMax[0] | float | `MEAS:WEIG:MAX:CH0`, power-down |
| 28 | backlightPWM | uint8_t | `SYST:BACK:PWM`, power-down |
| 29 | echo | uint8_t | `SYST:ECHO`, power-down |
| 30 | prompt | uint8_t | `SYST:PROM`, power-down |
| 31 | stabThreshold[0] | float | `CONF:STAB:THR:CH0`, power-down |
| 35 | overloadCapacity[0] | float | `CONF:OVER:CAP:CH0`, power-down |
| 39 | adaptiveFilterEnable[0] | uint8_t | `CONF:FILT:ADAP:CH0`, power-down |
| 40 | adaptiveFilterPct[0] | float | `CONF:FILT:ADAP:THR:CH0`, power-down |
| 44 | adaptiveFilterTimeUs[0] | uint32_t | `CONF:FILT:ADAP:TIME:CH0`, power-down |
| 48 | adcInvert[0] | uint8_t | `CONF:ADC:INV:CH0`, power-down |

### CH1 Per-Channel Settings

| Address | Field | Type | Persisted by |
|---------|-------|------|-------------|
| 49 | calValue[1] | float | `CAL:VAL:CH1`, `CAL:SPAN:EXEC:CH1`, calibration |
| 53 | zeroValue[1] | int32_t | `CAL:ZERO:CH1`, `CAL:ZERO:EXEC:CH1`, `CONF:ZERO:CH1`, calibration |
| 57 | unitVal[1] | enum (int) | `CONF:UNIT:CH1`, power-down |
| 61 | calWeight[1] | uint32_t | `CAL:WEIG:CH1`, calibration |
| 65 | calUnit[1] | enum (int) | `CAL:UNIT:CH1`, calibration |
| 69 | extADCweightMax[1] | float | `MEAS:WEIG:MAX:CH1`, power-down |
| 73 | overloadCapacity[1] | float | `CONF:OVER:CAP:CH1`, power-down |
| 77 | stabThreshold[1] | float | `CONF:STAB:THR:CH1`, power-down |
| 81 | adaptiveFilterEnable[1] | uint8_t | `CONF:FILT:ADAP:CH1`, power-down |
| 82 | adaptiveFilterPct[1] | float | `CONF:FILT:ADAP:THR:CH1`, power-down |
| 86 | adaptiveFilterTimeUs[1] | uint32_t | `CONF:FILT:ADAP:TIME:CH1`, power-down |
| 90 | adcInvert[1] | uint8_t | `CONF:ADC:INV:CH1`, power-down |

### Tail Settings

| Address | Field | Type | Persisted by |
|---------|-------|------|-------------|
| 91 | displayMode | uint8_t | `CONF:DISP:MODE`, power-down |

## Examples

```
# Identify the instrument
*IDN?
→ Tacuna Systems,TacunaScale,00000000,1.2.0a

# Read weight from each channel
MEAS:WEIG:CH0?
→ 25.4321
MEAS:WEIG:CH1?
→ 12.3456

# Read combined weight (CH1 converted to CH0's unit)
MEAS:WEIG:SUM?
→ 37.7777

# Tare channel 0
CONF:TARE:CH0

# Set CH0 to kilograms, CH1 to pounds (independent units)
CONF:UNIT:CH0 KG
CONF:UNIT:CH1 LB

# Query units
CONF:UNIT:CH0?
→ KG
CONF:UNIT:CH1?
→ LB

# Switch display mode
CONF:DISP:MODE DUAL
CONF:DISP:MODE?
→ DUAL
CONF:DISP:MODE SUM
CONF:DISP:MODE?
→ SUM
CONF:DISP:MODE SING
CONF:DISP:MODE?
→ SINGle

# Read all EEPROM values
SYST:EEPROM?
→ CH0:{calValue=7168.220215,zeroValue=8295856,unit=KG,calWeight=100,
   calUnit=KG,weightMax=25.4321,overCap=500.0000,stabThresh=0.0002,
   adaptEn=1,adaptThr=1.0000,adaptTimeUs=750000,adcInvert=0},
   CH1:{calValue=7168.220215,zeroValue=8295856,unit=LB,calWeight=25,
   calUnit=LB,weightMax=12.3456,overCap=500.0000,stabThresh=0.0002,
   adaptEn=1,adaptThr=1.0000,adaptTimeUs=750000,adcInvert=0},
   backlight=1,backlightPWM=31,echo=1,prompt=1

# Reset peak weight for CH0
MEAS:WEIG:MAX:CH0 0

# Check for errors
SYST:ERR?
→ 0,"No error"

# Remote calibration workflow (per-channel)
# --- Calibrate CH0 ---
CAL:UNIT:CH0 KG
CAL:WEIG:CH0 100
# (unload scale)
CAL:ZERO:EXEC:CH0
→ (blocks ~2.5 s, captures zero)
# (load 100 kg reference weight on CH0)
CAL:SPAN:EXEC:CH0
→ (blocks ~2.5 s, computes calValue)
MEAS:WEIG:CH0?
→ 100.0000

# --- Calibrate CH1 ---
CAL:UNIT:CH1 LB
CAL:WEIG:CH1 50
CAL:ZERO:EXEC:CH1
CAL:SPAN:EXEC:CH1
MEAS:WEIG:CH1?
→ 50.0000

# If readings show negative after calibration, the load cell
# produces decreasing counts under load — toggle ADC invert:
CONF:ADC:INV:CH1 ON
MEAS:WEIG:CH1?
→ 50.0000

# ADC tuning (shared — one physical ADC)
CONF:ADC:RATE 300
CONF:ADC:RATE?
→ 300
CONF:ADC:FILT SINC3
CONF:ADC:FILT?
→ SINC3

# Stability detection (per-channel)
MEAS:WEIG:SDEV:CH0?
→ 0.0234
MEAS:WEIG:STAB:CH0?
→ 1
CONF:STAB:THR:CH0 0.00001
MEAS:WEIG:STAB:CH0?
→ 0

# Gross weight (before tare)
CONF:TARE:CH0
MEAS:WEIG:CH0?
→ 0.0000
MEAS:WEIG:GROS:CH0?
→ 25.4321

# Direct tare set/clear
CONF:TARE:CH0 10.5
CONF:TARE:CH0?
→ 10.500000
CONF:TARE:CH0 0

# Overload detection (per-channel)
MEAS:WEIG:OVER:CH0?
→ 0
CONF:OVER:CAP:CH0 0.001
MEAS:WEIG:OVER:CH0?
→ 1
CONF:OVER:CAP:CH0 500

# ADC polarity inversion (per-channel)
CONF:ADC:INV:CH0?
→ 0
CONF:ADC:INV:CH0 ON
MEAS:WEIG:CH0?
→ -25.4321
CONF:ADC:INV:CH0 OFF

# Adaptive filter tuning (per-channel)
CONF:FILT:ADAP:CH0?
→ 1
CONF:FILT:ADAP:THR:CH0?
→ 1
CONF:FILT:ADAP:TIME:CH0?
→ 750000
CONF:FILT:ADAP:THR:CH0 0.5
CONF:FILT:ADAP:TIME:CH0 500000
CONF:FILT:ADAP:CH0 OFF
```
