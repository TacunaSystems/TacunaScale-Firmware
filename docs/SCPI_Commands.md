# TacunaScale SCPI Command Reference

**Firmware:** 1.2.0a | **Updated:** 2026-03-10

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
lowercase letters may be omitted. For example, `CONFigure:UNIT` can be sent as
`CONF:UNIT`. Both forms are equivalent.

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
| `MEASure:WEIGht?` | Averaged weight in current display unit | Float |
| `MEASure:WEIGht:RAW?` | Combined raw ADC counts (Ch0 + Ch1 or Ch0 only) | Int32 |
| `MEASure:WEIGht:RAW:CH0?` | Raw ADC counts, channel 0 | Int32 |
| `MEASure:WEIGht:RAW:CH1?` | Raw ADC counts, channel 1 | Int32 |
| `MEASure:WEIGht:MAX?` | Peak weight recorded since last reset | Float |
| `MEASure:WEIGht:MAX <val>` | Set/reset peak weight tracker (persists to EEPROM) | — |
| `MEASure:WEIGht:AVERage:COUNt?` | Samples currently in running average buffer | Int |
| `MEASure:WEIGht:AVERage:SIZE?` | Running average buffer size (compile-time, default 5) | Int |
| `MEASure:WEIGht:SDEViation?` | Standard deviation of running average buffer | Float |
| `MEASure:WEIGht:STABle?` | Stability flag: 1 = settled, 0 = unstable. Settled when std dev < (threshold x capacity) AND buffer full | Bool (0/1) |
| `MEASure:WEIGht:GROSS?` | Weight before tare subtraction, in current display unit | Float |
| `MEASure:WEIGht:OVERload?` | Overload flag: 1 = abs(weight) > capacity, 0 = OK | Bool (0/1) |

All `MEASure:WEIGht` commands are also available as `MEASure:FORCe` aliases
(e.g. `MEAS:FORC?` is equivalent to `MEAS:WEIG?`).

## Configuration Commands

### Units

| Token | Unit | Type | Conversions |
|-------|------|------|-------------|
| `KG` | Kilogram | Mass | KG ↔ LB, KG ↔ N |
| `LB` | Pound | Mass | LB ↔ KG, LB ↔ N |
| `N` | Newton | Force | N ↔ KG, N ↔ LB |
| `NM` | Newton-meter | Torque | NM ↔ LBFT |
| `LBFT` | Pound-foot | Torque | LBFT ↔ NM |

Switching between compatible units auto-converts overload capacity and peak
weight. Cross-type changes (e.g. mass → torque) change only the label.

| Command | Description | Parameter |
|---------|-------------|-----------|
| `CONFigure:UNIT <KG\|LB\|N\|NM\|LBFT>` | Set display unit | `KG`, `LB`, `N`, `NM`, or `LBFT` |
| `CONFigure:UNIT?` | Query current display unit | — |
| `CONFigure:TARE [<value>]` | Tare — no param: auto-tare from current reading. With param: set tare offset directly (in cal units) | Float (optional) |
| `CONFigure:TARE?` | Query current tare offset | — |
| `CONFigure:ZERO` | Set current ADC reading as zero reference (persists to EEPROM) | — |

### ADC Settings

| Command | Description | Parameter |
|---------|-------------|-----------|
| `CONFigure:ADC:RATE <1-1023>` | Set AD7193 filter output rate (runtime only) | Int |
| `CONFigure:ADC:RATE?` | Query current ADC rate setting | — |
| `CONFigure:ADC:FILTer <SINC3\|SINC4>` | Set digital filter type (runtime only) | `SINC3` or `SINC4` |
| `CONFigure:ADC:FILTer?` | Query current filter type | — |
| `CONFigure:ADC:NOTCh <ON\|OFF>` | Enable/disable 50/60 Hz notch rejection (runtime only) | Boolean |
| `CONFigure:ADC:NOTCh?` | Query notch filter state | — |
| `CONFigure:ADC:INVert <ON\|OFF>` | Invert ADC polarity (persists). Negates all raw and weight values. Calibration is preserved — no re-calibration required. | Boolean |
| `CONFigure:ADC:INVert?` | Query ADC inversion state | Bool (0/1) |

### Stability and Overload

| Command | Description | Parameter |
|---------|-------------|-----------|
| `CONFigure:STABility:THReshold <val>` | Stability threshold as fraction of capacity (persists, default 0.0002 = 0.02%) | Float (> 0) |
| `CONFigure:STABility:THReshold?` | Query current stability threshold | — |
| `CONFigure:OVERload:CAPacity <val>` | Overload capacity in display units (persists, default 500). Auto-converts when display unit changes. | Float (> 0) |
| `CONFigure:OVERload:CAPacity?` | Query current overload capacity | — |

### Adaptive Filter

| Command | Description | Parameter |
|---------|-------------|-----------|
| `CONFigure:FILTer:ADAPtive <ON\|OFF>` | Enable/disable adaptive filter (persists, default ON) | Boolean |
| `CONFigure:FILTer:ADAPtive?` | Query adaptive filter state | — |
| `CONFigure:FILTer:ADAPtive:THReshold <val>` | Deviation threshold as % of capacity (persists, default 1.0) | Float (> 0) |
| `CONFigure:FILTer:ADAPtive:THReshold?` | Query adaptive filter threshold | — |
| `CONFigure:FILTer:ADAPtive:TIME <val>` | Sustained deviation window in microseconds (persists, default 750000) | Int (1-60000000) |
| `CONFigure:FILTer:ADAPtive:TIME?` | Query adaptive filter time window | — |

The adaptive filter detects sustained directional weight changes (e.g. a new
load placed on the scale) and clears the running average for immediate display
response. Oscillatory motion (e.g. a patient shifting on a chair scale) resets
the detection timer, so normal averaging continues undisturbed.

Each new reading is compared to the current average. If the deviation exceeds
`threshold x capacity / 100` and persists in the same direction for longer than
the configured time window, the running average buffer is cleared. The time-based
window automatically scales with sample rate.

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
| `CALibration:UNIT <KG\|LB\|N\|NM\|LBFT>` | Set calibration unit | `KG`, `LB`, `N`, `NM`, or `LBFT` |
| `CALibration:ZERO:EXEC` | Block until settled (~2.5 s), capture zero reference, persist | — |
| `CALibration:SPAN:EXEC` | Block until settled (~2.5 s), auto-compute calValue, persist | — |

`CALibration:WEIGht` and `CALibration:WEIGht?` are also available as
`CALibration:FORCe` aliases.

## System Commands

| Command | Description | Parameter |
|---------|-------------|-----------|
| `SYSTem:BACKlight <ON\|OFF>` | Turn LCD backlight on/off | Boolean |
| `SYSTem:BACKlight?` | Query backlight state | — |
| `SYSTem:BACKlight:PWM <0-100>` | Set backlight PWM duty cycle (percent, persists) | Int (0-100) |
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
| `SYSTem:CONFig:SW1?` | DIP switch 1 state (read at boot) | Bool (0/1) |
| `SYSTem:CONFig:SW2?` | DIP switch 2 state (read at boot) | Bool (0/1) |
| `SYSTem:FW?` | Firmware version string | String |
| `SYSTem:EEPROM?` | Dump all persisted EEPROM values (comma-separated key=value) | — |
| `SYSTem:EEPROM:COMMit` | Flush pending EEPROM writes to flash | — |
| `SYSTem:LOG?` | Read debug log from RAM ring buffer (2 KB) | — |
| `SYSTem:LOG:CLEar` | Clear the debug log | — |

## EEPROM Map

All values persist across power cycles. The `SYSTem:EEPROM?` command reads all
fields directly from flash.

| Address | Field | Type | Persisted by |
|---------|-------|------|-------------|
| 0 | calValue | float | `CAL:VAL`, `CAL:SPAN:EXEC`, calibration routine |
| 4 | zeroValue | int32_t | `CAL:ZERO`, `CAL:ZERO:EXEC`, `CAL:SPAN:EXEC`, `CONF:ZERO`, calibration routine |
| 8 | backlightEnable | enum (int) | Power-down |
| 12 | unitVal | enum (int) | `CONF:OVER:CAP`, power-down |
| 16 | calWeight | uint32_t | `CAL:WEIG`, calibration routine |
| 20 | calUnit | enum (int) | `CAL:UNIT`, calibration routine |
| 24 | extADCweightMax | float | `MEAS:WEIG:MAX`, power-down |
| 28 | backlightPWM | uint8_t | `SYST:BACK:PWM`, power-down |
| 29 | echo | uint8_t | `SYST:ECHO`, power-down |
| 30 | prompt | uint8_t | `SYST:PROM`, power-down |
| 31 | stabThreshold | float | `CONF:STAB:THR`, power-down |
| 35 | overloadCapacity | float | `CONF:OVER:CAP`, power-down |
| 39 | adaptiveFilterEnable | uint8_t | `CONF:FILT:ADAP`, power-down |
| 40 | adaptiveFilterPct | float | `CONF:FILT:ADAP:THR`, power-down |
| 44 | adaptiveFilterTimeUs | uint32_t | `CONF:FILT:ADAP:TIME`, power-down |
| 48 | adcInvert | uint8_t | `CONF:ADC:INV`, power-down |

## Examples

```
# Identify the instrument
*IDN?
→ Tacuna Systems,TacunaScale,00000000,1.2.0a

# Read weight
MEAS:WEIG?
→ 25.4321

# Tare the scale
CONF:TARE

# Switch to pounds
CONF:UNIT LB

# Read all EEPROM values
SYST:EEPROM?
→ calValue=7168.220215,zeroValue=8295856,backlight=1,unit=KG,
   calWeight=100,calUnit=KG,weightMax=25.4321,backlightPWM=31,
   echo=1,prompt=1,stabThresh=0.0002,overCap=500.0000,
   adaptEn=1,adaptThr=1.0000,adaptTimeUs=750000,adcInvert=0

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
→ (blocks ~2.5 s, captures zero)
# (load 100 kg reference weight)
CAL:SPAN:EXEC
→ (blocks ~2.5 s, computes calValue)
MEAS:WEIG?
→ 100.0000

# ADC tuning
CONF:ADC:RATE 300
CONF:ADC:RATE?
→ 300
CONF:ADC:FILT SINC3
CONF:ADC:FILT?
→ SINC3

# Stability detection
# Threshold is a fraction of overload capacity (default 0.0002 = 0.02%)
# With 500 lb capacity: stable when sdev < 0.0002 × 500 = 0.1 lb
MEAS:WEIG:SDEV?
→ 0.0234
MEAS:WEIG:STAB?
→ 1
CONF:STAB:THR 0.00001
MEAS:WEIG:STAB?
→ 0

# Gross weight (before tare)
CONF:TARE
MEAS:WEIG?
→ 0.0000
MEAS:WEIG:GROSS?
→ 25.4321

# Direct tare set/clear
CONF:TARE 10.5
CONF:TARE?
→ 10.500000
CONF:TARE 0

# Overload detection
MEAS:WEIG:OVER?
→ 0
CONF:OVER:CAP 0.001
MEAS:WEIG:OVER?
→ 1
CONF:OVER:CAP 500

# ADC polarity inversion
# Inverts all raw ADC values and derived weights.
# Zero, tare, and peak are automatically adjusted — no re-calibration needed.
CONF:ADC:INV?
→ 0
CONF:ADC:INV ON
MEAS:WEIG?
→ -25.4321
CONF:ADC:INV OFF

# Adaptive filter tuning
CONF:FILT:ADAP?
→ 1
CONF:FILT:ADAP:THR?
→ 1
CONF:FILT:ADAP:TIME?
→ 750000
# Tighten threshold to 0.5% of capacity
CONF:FILT:ADAP:THR 0.5
# Shorten detection window to 500 ms
CONF:FILT:ADAP:TIME 500000
# Disable adaptive filter (pure running average)
CONF:FILT:ADAP OFF
```
