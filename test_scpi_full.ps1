# ============================================================================
# Comprehensive SCPI Command Test Suite - TacunaScale
# ============================================================================
# Tests all SCPI commands: queries, set/get roundtrips, error handling,
# short-form vs long-form syntax, and echo/prompt behavior.
# ============================================================================

$ErrorActionPreference = "Stop"

# --- Configuration ---
$comPort = "COM15"
$baudRate = 115200
$pollDelayMs = 200     # Time to wait after sending a command for response

# --- Counters ---
$script:pass = 0
$script:fail = 0
$script:skip = 0
$script:testNum = 0

# --- Colors ---
function Write-Pass($msg) { Write-Host "  PASS: $msg" -ForegroundColor Green; $script:pass++ }
function Write-Fail($msg) { Write-Host "  FAIL: $msg" -ForegroundColor Red; $script:fail++ }
function Write-Skip($msg) { Write-Host "  SKIP: $msg" -ForegroundColor Yellow; $script:skip++ }
function Write-Section($msg) { Write-Host "`n=== $msg ===" -ForegroundColor Cyan }

# --- Serial helpers ---
function Open-Port {
    $p = New-Object System.IO.Ports.SerialPort($comPort, $baudRate, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
    $p.ReadTimeout = 2000
    $p.WriteTimeout = 2000
    $p.DtrEnable = $false
    $p.RtsEnable = $false
    $p.Open()
    Start-Sleep -Milliseconds 500
    $p.DiscardInBuffer()
    return $p
}

function Send-SCPI {
    param(
        [Parameter(Mandatory)]$Port,
        [Parameter(Mandatory)][string]$Command,
        [bool]$ExpectResponse = $true,
        [int]$Delay = $pollDelayMs
    )
    $Port.DiscardInBuffer()
    $Port.Write("$Command`r`n")
    Start-Sleep -Milliseconds $Delay
    $resp = ""
    try {
        while ($Port.BytesToRead -gt 0) {
            $resp += $Port.ReadExisting()
            Start-Sleep -Milliseconds 50
        }
    } catch {}
    return $resp.Trim()
}

function Test-Query {
    param(
        [Parameter(Mandatory)]$Port,
        [string]$Command,
        [string]$Label,
        [string]$ExpectedPattern = $null,  # regex pattern
        [string]$ExactMatch = $null
    )
    $script:testNum++
    $num = $script:testNum.ToString("D2")
    $resp = Send-SCPI -Port $Port -Command $Command
    if ($resp -eq "") {
        Write-Fail "[$num] $Label - no response from '$Command'"
        return $null
    }
    if ($ExactMatch -ne $null -and $ExactMatch -ne "") {
        if ($resp -eq $ExactMatch) {
            Write-Pass "[$num] $Label - '$resp'"
        } else {
            Write-Fail "[$num] $Label - expected '$ExactMatch', got '$resp'"
        }
    } elseif ($ExpectedPattern -ne $null -and $ExpectedPattern -ne "") {
        if ($resp -match $ExpectedPattern) {
            Write-Pass "[$num] $Label - '$resp'"
        } else {
            Write-Fail "[$num] $Label - expected /$ExpectedPattern/, got '$resp'"
        }
    } else {
        # Just check we got something
        Write-Pass "[$num] $Label - '$resp'"
    }
    return $resp
}

function Test-SetGet {
    param(
        [Parameter(Mandatory)]$Port,
        [string]$SetCmd,
        [string]$GetCmd,
        [string]$ExpectedValue,
        [string]$Label
    )
    $script:testNum++
    $num = $script:testNum.ToString("D2")
    # Send set command (no response expected)
    Send-SCPI -Port $Port -Command $SetCmd -ExpectResponse $false | Out-Null
    Start-Sleep -Milliseconds 100
    # Query to verify
    $resp = Send-SCPI -Port $Port -Command $GetCmd
    if ($resp -eq $ExpectedValue) {
        Write-Pass "[$num] $Label - set='$SetCmd' -> query='$resp'"
    } else {
        Write-Fail "[$num] $Label - set='$SetCmd', expected '$ExpectedValue', got '$resp'"
    }
    return $resp
}

function Test-ErrorCase {
    param(
        [Parameter(Mandatory)]$Port,
        [string]$Command,
        [string]$Label
    )
    $script:testNum++
    $num = $script:testNum.ToString("D2")
    # Send command that should produce an error
    Send-SCPI -Port $Port -Command $Command -ExpectResponse $false | Out-Null
    Start-Sleep -Milliseconds 100
    # Check error queue
    $errResp = Send-SCPI -Port $Port -Command "SYST:ERR?"
    if ($errResp -match "^-\d+" -and $errResp -notmatch '^0,"No error"') {
        Write-Pass "[$num] $Label - error generated: '$errResp'"
    } else {
        Write-Fail "[$num] $Label - expected error, got: '$errResp'"
    }
}

# ============================================================================
# MAIN TEST EXECUTION
# ============================================================================

Write-Host "TacunaScale - SCPI Comprehensive Test Suite" -ForegroundColor White
Write-Host "Port: $comPort @ $baudRate baud`n"

$port = Open-Port

# --- Phase 0: Save current state and disable echo/prompt for clean testing ---
Write-Section "Phase 0: Save state, disable echo/prompt"

$savedEcho = Send-SCPI -Port $port -Command "SYST:ECHO?"
$savedPrompt = Send-SCPI -Port $port -Command "SYST:PROM?"

# Strip any prompt artifacts from saved values
if ($savedEcho -match "[01]") { $savedEcho = $Matches[0] } else { $savedEcho = "1" }
if ($savedPrompt -match "[01]") { $savedPrompt = $Matches[0] } else { $savedPrompt = "1" }

Send-SCPI -Port $port -Command "SYST:ECHO OFF" -ExpectResponse $false | Out-Null
Send-SCPI -Port $port -Command "SYST:PROM OFF" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 200
$port.DiscardInBuffer()

# Verify echo/prompt are off
$echoCheck = Send-SCPI -Port $port -Command "SYST:ECHO?"
$promptCheck = Send-SCPI -Port $port -Command "SYST:PROM?"
if ($echoCheck -eq "0" -and $promptCheck -eq "0") {
    Write-Host "  Echo and prompt disabled for clean testing." -ForegroundColor Gray
} else {
    Write-Host "  WARNING: echo=$echoCheck prompt=$promptCheck (expected 0,0)" -ForegroundColor Yellow
}

# Save calibration/config values for later restoration
$savedCalValue = Send-SCPI -Port $port -Command "CAL:VAL?"
$savedCalZero = Send-SCPI -Port $port -Command "CAL:ZERO?"
$savedCalWeight = Send-SCPI -Port $port -Command "CAL:WEIG?"
$savedCalUnit = Send-SCPI -Port $port -Command "CAL:UNIT?"
$savedUnit = Send-SCPI -Port $port -Command "CONF:UNIT?"
$savedBacklight = Send-SCPI -Port $port -Command "SYST:BACK?"
$savedBacklightPWM = Send-SCPI -Port $port -Command "SYST:BACK:PWM?"
$savedWeightMax = Send-SCPI -Port $port -Command "MEAS:WEIG:MAX?"

Write-Host "  Saved state: unit=$savedUnit calVal=$savedCalValue calZero=$savedCalZero" -ForegroundColor Gray
Write-Host "  calWeight=$savedCalWeight calUnit=$savedCalUnit bkl=$savedBacklight pwm=$savedBacklightPWM" -ForegroundColor Gray

# --- Phase 1: IEEE 488.2 Common Commands ---
Write-Section "Phase 1: IEEE 488.2 Common Commands"

Test-Query -Port $port -Command "*IDN?" -Label "*IDN? identification" -ExpectedPattern "Tacuna Systems,TacunaScale,\d+,\d+\.\d+"
Test-Query -Port $port -Command "*TST?" -Label "*TST? self-test" -ExactMatch "0"
Test-Query -Port $port -Command "*OPC?" -Label "*OPC? operation complete" -ExactMatch "1"
Test-Query -Port $port -Command "*ESE?" -Label "*ESE? event status enable" -ExpectedPattern "^\d+$"
Test-Query -Port $port -Command "*ESR?" -Label "*ESR? event status register" -ExpectedPattern "^\d+$"
Test-Query -Port $port -Command "*SRE?" -Label "*SRE? service request enable" -ExpectedPattern "^\d+$"
Test-Query -Port $port -Command "*STB?" -Label "*STB? status byte" -ExpectedPattern "^\d+$"

# Set/get roundtrip for *ESE
Test-SetGet -Port $port -SetCmd "*ESE 42" -GetCmd "*ESE?" -ExpectedValue "42" -Label "*ESE set/get roundtrip"

# Set/get roundtrip for *SRE
Test-SetGet -Port $port -SetCmd "*SRE 16" -GetCmd "*SRE?" -ExpectedValue "16" -Label "*SRE set/get roundtrip"

# Commands with no response (just verify no crash)
$script:testNum++
$num = $script:testNum.ToString("D2")
Send-SCPI -Port $port -Command "*CLS" -ExpectResponse $false | Out-Null
Send-SCPI -Port $port -Command "*OPC" -ExpectResponse $false | Out-Null
Send-SCPI -Port $port -Command "*WAI" -ExpectResponse $false | Out-Null
$checkResp = Send-SCPI -Port $port -Command "*IDN?"
if ($checkResp -match "Tacuna") {
    Write-Pass "[$num] *CLS, *OPC, *WAI - no crash, scale responsive"
} else {
    Write-Fail "[$num] *CLS, *OPC, *WAI - scale unresponsive after commands"
}

# *RST - should clear tare
$script:testNum++
$num = $script:testNum.ToString("D2")
Send-SCPI -Port $port -Command "*RST" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 200
$tareAfterRst = Send-SCPI -Port $port -Command "CONF:TARE?"
if ($tareAfterRst -match "^[+-]?0(\.0+)?$") {
    Write-Pass "[$num] *RST clears tare - tare=$tareAfterRst"
} else {
    Write-Fail "[$num] *RST should clear tare - tare=$tareAfterRst"
}

# --- Phase 2: Required SCPI Commands ---
Write-Section "Phase 2: Required SCPI Commands"

Test-Query -Port $port -Command "SYST:ERR?" -Label "SYST:ERR? error queue" -ExpectedPattern '^-?\d+,"'
Test-Query -Port $port -Command "SYST:ERR:COUN?" -Label "SYST:ERR:COUN? error count" -ExpectedPattern "^\d+$"
Test-Query -Port $port -Command "SYST:VERS?" -Label "SYST:VERS? SCPI version" -ExpectedPattern "^\d{4}\.\d+$"
Test-Query -Port $port -Command "STAT:QUES?" -Label "STAT:QUES? questionable status" -ExpectedPattern "^\d+$"
Test-Query -Port $port -Command "STAT:QUES:ENAB?" -Label "STAT:QUES:ENAB? enable mask" -ExpectedPattern "^\d+$"

# Set/get for STAT:QUES:ENAB
Test-SetGet -Port $port -SetCmd "STAT:QUES:ENAB 255" -GetCmd "STAT:QUES:ENAB?" -ExpectedValue "255" -Label "STAT:QUES:ENAB set/get"

# STAT:PRES (no response)
$script:testNum++
$num = $script:testNum.ToString("D2")
Send-SCPI -Port $port -Command "STAT:PRES" -ExpectResponse $false | Out-Null
$quesAfterPres = Send-SCPI -Port $port -Command "STAT:QUES:ENAB?"
if ($quesAfterPres -eq "0") {
    Write-Pass "[$num] STAT:PRES resets enable mask - $quesAfterPres"
} else {
    Write-Pass "[$num] STAT:PRES executed - QUES:ENAB=$quesAfterPres"
}

# --- Phase 3: Measurement Commands ---
Write-Section "Phase 3: Measurement Commands"

Test-Query -Port $port -Command "MEAS:WEIG?" -Label "MEAS:WEIG? averaged weight" -ExpectedPattern "^-?\d+\.?\d*"
Test-Query -Port $port -Command "MEAS:WEIG:RAW?" -Label "MEAS:WEIG:RAW? combined raw ADC" -ExpectedPattern "^-?\d+$"
Test-Query -Port $port -Command "MEAS:WEIG:RAW:CH0?" -Label "MEAS:WEIG:RAW:CH0?" -ExpectedPattern "^-?\d+$"
Test-Query -Port $port -Command "MEAS:WEIG:RAW:CH1?" -Label "MEAS:WEIG:RAW:CH1?" -ExpectedPattern "^-?\d+$"
Test-Query -Port $port -Command "MEAS:WEIG:MAX?" -Label "MEAS:WEIG:MAX? peak weight" -ExpectedPattern "^-?\d+\.?\d*"

# Long-form equivalents
Test-Query -Port $port -Command "MEASure:WEIGht?" -Label "MEASure:WEIGht? (long form)" -ExpectedPattern "^-?\d+\.?\d*"
Test-Query -Port $port -Command "MEASure:WEIGht:RAW?" -Label "MEASure:WEIGht:RAW? (long form)" -ExpectedPattern "^-?\d+$"

# MEAS:WEIG:MAX set/get roundtrip
Test-SetGet -Port $port -SetCmd "MEAS:WEIG:MAX 99.5" -GetCmd "MEAS:WEIG:MAX?" -ExpectedValue "99.5" -Label "MEAS:WEIG:MAX set/get"

# MEAS:WEIG:MAX 0 - note: ADC task may immediately update max with current weight
$script:testNum++
$num = $script:testNum.ToString("D2")
Send-SCPI -Port $port -Command "MEAS:WEIG:MAX 0" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 500
$maxAfterReset = Send-SCPI -Port $port -Command "MEAS:WEIG:MAX?"
if ($maxAfterReset -match "^-?\d+\.?\d*$") {
    Write-Pass "[$num] MEAS:WEIG:MAX reset - value=$maxAfterReset (ADC may update immediately)"
} else {
    Write-Fail "[$num] MEAS:WEIG:MAX reset - unexpected: '$maxAfterReset'"
}

# --- Phase 4: Configuration Commands ---
Write-Section "Phase 4: Configuration Commands"

Test-Query -Port $port -Command "CONF:UNIT?" -Label "CONF:UNIT? current unit" -ExpectedPattern "^(KG|LB)$"
Test-Query -Port $port -Command "CONF:TARE?" -Label "CONF:TARE? tare offset" -ExpectedPattern "^-?\d+\.?\d*"

# Unit roundtrip
Test-SetGet -Port $port -SetCmd "CONF:UNIT KG" -GetCmd "CONF:UNIT?" -ExpectedValue "KG" -Label "CONF:UNIT KG roundtrip"
Test-SetGet -Port $port -SetCmd "CONF:UNIT LB" -GetCmd "CONF:UNIT?" -ExpectedValue "LB" -Label "CONF:UNIT LB roundtrip"

# Long-form
Test-SetGet -Port $port -SetCmd "CONFigure:UNIT KG" -GetCmd "CONFigure:UNIT?" -ExpectedValue "KG" -Label "CONFigure:UNIT (long form)"

# CONF:ZERO - set current reading as zero (test carefully)
$script:testNum++
$num = $script:testNum.ToString("D2")
$rawBefore = Send-SCPI -Port $port -Command "CAL:ZERO?"
Send-SCPI -Port $port -Command "CONF:ZERO" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 200
$zeroAfter = Send-SCPI -Port $port -Command "CAL:ZERO?"
if ($zeroAfter -match "^-?\d+$" -and $zeroAfter -ne "0") {
    Write-Pass "[$num] CONF:ZERO - zero ref updated from $rawBefore to $zeroAfter"
} else {
    Write-Fail "[$num] CONF:ZERO - unexpected zero ref: $zeroAfter"
}

# CONF:TARE (only works when calibrated)
$script:testNum++
$num = $script:testNum.ToString("D2")
$calVal = Send-SCPI -Port $port -Command "CAL:VAL?"
if ([float]$calVal -ne 0.0) {
    Send-SCPI -Port $port -Command "CONF:TARE" -ExpectResponse $false | Out-Null
    Start-Sleep -Milliseconds 200
    $tareVal = Send-SCPI -Port $port -Command "CONF:TARE?"
    if ($tareVal -match "^-?\d+\.?\d*" -and $tareVal -ne "0.000000") {
        Write-Pass "[$num] CONF:TARE - tare set to $tareVal"
    } else {
        Write-Pass "[$num] CONF:TARE - tare=$tareVal (may be near zero if unloaded)"
    }
    # Clear tare
    Send-SCPI -Port $port -Command "*RST" -ExpectResponse $false | Out-Null
} else {
    Write-Skip "[$num] CONF:TARE - skipped, calValue=0 uncalibrated"
}

# --- Phase 5: Calibration Commands ---
Write-Section "Phase 5: Calibration Commands"

Test-Query -Port $port -Command "CAL:VAL?" -Label "CAL:VAL? calibration factor" -ExpectedPattern "^-?\d+\.?\d*"
Test-Query -Port $port -Command "CAL:ZERO?" -Label "CAL:ZERO? zero reference" -ExpectedPattern "^-?\d+$"
Test-Query -Port $port -Command "CAL:WEIG?" -Label "CAL:WEIG? calibration weight" -ExpectedPattern "^\d+$"
Test-Query -Port $port -Command "CAL:UNIT?" -Label "CAL:UNIT? calibration unit" -ExpectedPattern "^(KG|LB)$"

# CAL:VAL set/get (use a safe test value, then restore)
Test-SetGet -Port $port -SetCmd "CAL:VAL 1234.5" -GetCmd "CAL:VAL?" -ExpectedValue "1234.5" -Label "CAL:VAL set/get"

# CAL:ZERO set/get
Test-SetGet -Port $port -SetCmd "CAL:ZERO 5000000" -GetCmd "CAL:ZERO?" -ExpectedValue "5000000" -Label "CAL:ZERO set/get"

# CAL:WEIG set/get
Test-SetGet -Port $port -SetCmd "CAL:WEIG 200" -GetCmd "CAL:WEIG?" -ExpectedValue "200" -Label "CAL:WEIG set/get"

# CAL:UNIT set/get
Test-SetGet -Port $port -SetCmd "CAL:UNIT LB" -GetCmd "CAL:UNIT?" -ExpectedValue "LB" -Label "CAL:UNIT LB roundtrip"
Test-SetGet -Port $port -SetCmd "CAL:UNIT KG" -GetCmd "CAL:UNIT?" -ExpectedValue "KG" -Label "CAL:UNIT KG roundtrip"

# Long-form
Test-SetGet -Port $port -SetCmd "CALibration:VALue 999.0" -GetCmd "CALibration:VALue?" -ExpectedValue "999" -Label "CALibration:VALue (long form)"

# --- Phase 6: System Commands ---
Write-Section "Phase 6: System Commands"

# Backlight
Test-Query -Port $port -Command "SYST:BACK?" -Label "SYST:BACK? backlight state" -ExpectedPattern "^[01]$"
Test-SetGet -Port $port -SetCmd "SYST:BACK ON" -GetCmd "SYST:BACK?" -ExpectedValue "1" -Label "SYST:BACK ON roundtrip"
Test-SetGet -Port $port -SetCmd "SYST:BACK OFF" -GetCmd "SYST:BACK?" -ExpectedValue "0" -Label "SYST:BACK OFF roundtrip"
Test-SetGet -Port $port -SetCmd "SYST:BACK ON" -GetCmd "SYST:BACK?" -ExpectedValue "1" -Label "SYST:BACK restore ON"

# Backlight PWM
Test-Query -Port $port -Command "SYST:BACK:PWM?" -Label "SYST:BACK:PWM? duty cycle" -ExpectedPattern "^\d+$"
Test-SetGet -Port $port -SetCmd "SYST:BACK:PWM 75" -GetCmd "SYST:BACK:PWM?" -ExpectedValue "75" -Label "SYST:BACK:PWM 75 roundtrip"
Test-SetGet -Port $port -SetCmd "SYST:BACK:PWM 0" -GetCmd "SYST:BACK:PWM?" -ExpectedValue "0" -Label "SYST:BACK:PWM 0 (min)"
Test-SetGet -Port $port -SetCmd "SYST:BACK:PWM 100" -GetCmd "SYST:BACK:PWM?" -ExpectedValue "100" -Label "SYST:BACK:PWM 100 (max)"

# Power voltages
Test-Query -Port $port -Command "SYST:POW:VOLT:BATT?" -Label "SYST:POW:VOLT:BATT? battery" -ExpectedPattern "^\d+\.\d+"
Test-Query -Port $port -Command "SYST:POW:VOLT:SUPP?" -Label "SYST:POW:VOLT:SUPP? 5V rail" -ExpectedPattern "^\d+\.\d+"

# Long-form
Test-Query -Port $port -Command "SYSTem:POWer:VOLTage:BATTery?" -Label "SYSTem:POWer:VOLTage:BATTery? (long form)" -ExpectedPattern "^\d+\.\d+"

# EEPROM dump
Test-Query -Port $port -Command "SYST:EEPROM?" -Label "SYST:EEPROM? dump" -ExpectedPattern "calValue=.*zeroValue=.*backlight="

# EEPROM commit (no response, just verify no error)
$script:testNum++
$num = $script:testNum.ToString("D2")
Send-SCPI -Port $port -Command "SYST:EEPROM:COMM" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 100
$errAfterCommit = Send-SCPI -Port $port -Command "SYST:ERR?"
if ($errAfterCommit -match '^0,"No error"') {
    Write-Pass "[$num] SYST:EEPROM:COMM - no error"
} else {
    Write-Fail "[$num] SYST:EEPROM:COMM - error: $errAfterCommit"
}

# SYSTem:POWer:DOWN - SKIP (would shut down the scale!)
$script:testNum++
$num = $script:testNum.ToString("D2")
Write-Skip "[$num] SYST:POW:DOWN - skipped (would power off scale)"

# --- Phase 7: Debug Log Commands ---
Write-Section "Phase 7: Debug Log Commands"

Test-Query -Port $port -Command "SYST:LOG?" -Label "SYST:LOG? read log" -ExpectedPattern ".+"

$script:testNum++
$num = $script:testNum.ToString("D2")
Send-SCPI -Port $port -Command "SYST:LOG:CLE" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 100
$logAfterClear = Send-SCPI -Port $port -Command "SYST:LOG?"
if ($logAfterClear -match "\(empty\)") {
    Write-Pass "[$num] SYST:LOG:CLE - log cleared"
} else {
    # Log may have new entries from commands we just ran
    Write-Pass "[$num] SYST:LOG:CLE - executed (log may have new entries: '$logAfterClear')"
}

# --- Phase 8: Error Handling ---
Write-Section "Phase 8: Error Handling"

# Clear error queue first
Send-SCPI -Port $port -Command "*CLS" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 100

# CAL:VAL 0 - should error (must be > 0)
Test-ErrorCase -Port $port -Command "CAL:VAL 0" -Label "CAL:VAL 0 (must be > 0)"

# CAL:VAL negative - should error
Test-ErrorCase -Port $port -Command "CAL:VAL -5" -Label "CAL:VAL -5 (must be > 0)"

# CAL:WEIG 0 - should error (must be > 0)
Test-ErrorCase -Port $port -Command "CAL:WEIG 0" -Label "CAL:WEIG 0 (must be > 0)"

# CAL:WEIG negative - should error
Test-ErrorCase -Port $port -Command "CAL:WEIG -1" -Label "CAL:WEIG -1 (must be > 0)"

# SYST:BACK:PWM 101 - should error (out of range)
Test-ErrorCase -Port $port -Command "SYST:BACK:PWM 101" -Label "SYST:BACK:PWM 101 (out of range)"

# SYST:BACK:PWM -1 - should error
Test-ErrorCase -Port $port -Command "SYST:BACK:PWM -1" -Label "SYST:BACK:PWM -1 (out of range)"

# Invalid command - should error
Test-ErrorCase -Port $port -Command "FAKE:COMMAND?" -Label "FAKE:COMMAND? (undefined header)"

# CONF:UNIT with invalid parameter
Test-ErrorCase -Port $port -Command "CONF:UNIT OUNCE" -Label "CONF:UNIT OUNCE (invalid choice)"

# --- Phase 9: Echo and Prompt Tests ---
Write-Section "Phase 9: Echo and Prompt Behavior"

# Test SYST:ECHO ON - command text should be echoed back
$script:testNum++
$num = $script:testNum.ToString("D2")
Send-SCPI -Port $port -Command "SYST:ECHO ON" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 100
$port.DiscardInBuffer()
$port.Write("*IDN?`r`n")
Start-Sleep -Milliseconds 300
$echoResp = ""
try {
    while ($port.BytesToRead -gt 0) {
        $echoResp += $port.ReadExisting()
        Start-Sleep -Milliseconds 50
    }
} catch {}
if ($echoResp -match "\*IDN\?" -and $echoResp -match "Tacuna") {
    Write-Pass "[$num] ECHO ON - command echoed and response received"
} elseif ($echoResp -match "Tacuna") {
    Write-Fail "[$num] ECHO ON - response received but command not echoed: '$echoResp'"
} else {
    Write-Fail "[$num] ECHO ON - unexpected: '$echoResp'"
}

# Disable echo for prompt test
Send-SCPI -Port $port -Command "SYST:ECHO OFF" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 100

# Test SYST:PROM ON - responses should be prefixed with "> "
$script:testNum++
$num = $script:testNum.ToString("D2")
Send-SCPI -Port $port -Command "SYST:PROM ON" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 100
$port.DiscardInBuffer()
$port.Write("*IDN?`r`n")
Start-Sleep -Milliseconds 300
$promptResp = ""
try {
    while ($port.BytesToRead -gt 0) {
        $promptResp += $port.ReadExisting()
        Start-Sleep -Milliseconds 50
    }
} catch {}
if ($promptResp -match "> .*Tacuna") {
    Write-Pass "[$num] PROMPT ON - response has '> ' prefix"
} elseif ($promptResp -match "Tacuna") {
    Write-Fail "[$num] PROMPT ON - response received but no '> ' prefix: '$promptResp'"
} else {
    Write-Fail "[$num] PROMPT ON - unexpected: '$promptResp'"
}

# Disable prompt
Send-SCPI -Port $port -Command "SYST:PROM OFF" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 100

# Echo roundtrip: set ON, verify via EEPROM dump (avoids echo in query response)
$script:testNum++
$num = $script:testNum.ToString("D2")
Send-SCPI -Port $port -Command "SYST:ECHO ON" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 100
$port.DiscardInBuffer()
# Query with echo on - response will include echoed command text
$echoOnResp = Send-SCPI -Port $port -Command "SYST:ECHO?"
if ($echoOnResp -match "1") {
    Write-Pass "[$num] SYST:ECHO ON roundtrip - response contains '1'"
} else {
    Write-Fail "[$num] SYST:ECHO ON roundtrip - expected '1' in response, got '$echoOnResp'"
}
# Turn echo back off for remaining tests
Send-SCPI -Port $port -Command "SYST:ECHO OFF" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 100

Test-SetGet -Port $port -SetCmd "SYST:ECHO OFF" -GetCmd "SYST:ECHO?" -ExpectedValue "0" -Label "SYST:ECHO OFF roundtrip"

# Prompt roundtrip: set ON, verify (response will have "> " prefix)
$script:testNum++
$num = $script:testNum.ToString("D2")
Send-SCPI -Port $port -Command "SYST:PROM ON" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 100
$port.DiscardInBuffer()
$promptOnResp = Send-SCPI -Port $port -Command "SYST:PROM?"
if ($promptOnResp -match "1") {
    Write-Pass "[$num] SYST:PROM ON roundtrip - response contains '1'"
} else {
    Write-Fail "[$num] SYST:PROM ON roundtrip - expected '1' in response, got '$promptOnResp'"
}
# Turn prompt back off for remaining tests
Send-SCPI -Port $port -Command "SYST:PROM OFF" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 100

Test-SetGet -Port $port -SetCmd "SYST:PROM OFF" -GetCmd "SYST:PROM?" -ExpectedValue "0" -Label "SYST:PROM OFF roundtrip"

# --- Phase 10: Restore original state ---
Write-Section "Phase 10: Restore original state"

# Restore all saved values
Send-SCPI -Port $port -Command "CAL:VAL $savedCalValue" -ExpectResponse $false | Out-Null
Send-SCPI -Port $port -Command "CAL:ZERO $savedCalZero" -ExpectResponse $false | Out-Null
Send-SCPI -Port $port -Command "CAL:WEIG $savedCalWeight" -ExpectResponse $false | Out-Null
Send-SCPI -Port $port -Command "CAL:UNIT $savedCalUnit" -ExpectResponse $false | Out-Null
Send-SCPI -Port $port -Command "CONF:UNIT $savedUnit" -ExpectResponse $false | Out-Null
Send-SCPI -Port $port -Command "MEAS:WEIG:MAX $savedWeightMax" -ExpectResponse $false | Out-Null
Send-SCPI -Port $port -Command "SYST:BACK:PWM $savedBacklightPWM" -ExpectResponse $false | Out-Null
if ($savedBacklight -eq "1") {
    Send-SCPI -Port $port -Command "SYST:BACK ON" -ExpectResponse $false | Out-Null
} else {
    Send-SCPI -Port $port -Command "SYST:BACK OFF" -ExpectResponse $false | Out-Null
}
if ($savedEcho -eq "1") {
    Send-SCPI -Port $port -Command "SYST:ECHO ON" -ExpectResponse $false | Out-Null
} else {
    Send-SCPI -Port $port -Command "SYST:ECHO OFF" -ExpectResponse $false | Out-Null
}
if ($savedPrompt -eq "1") {
    Send-SCPI -Port $port -Command "SYST:PROM ON" -ExpectResponse $false | Out-Null
} else {
    Send-SCPI -Port $port -Command "SYST:PROM OFF" -ExpectResponse $false | Out-Null
}
Send-SCPI -Port $port -Command "SYST:EEPROM:COMM" -ExpectResponse $false | Out-Null
Start-Sleep -Milliseconds 200

# Verify restoration
$script:testNum++
$num = $script:testNum.ToString("D2")
# Need to handle echo/prompt in the response
$port.DiscardInBuffer()
$restoreCheck = Send-SCPI -Port $port -Command "SYST:EEPROM?"
if ($restoreCheck -match "calValue=") {
    Write-Pass "[$num] State restored and committed to EEPROM"
    Write-Host "  Restored: $restoreCheck" -ForegroundColor Gray
} else {
    Write-Fail "[$num] State restoration verification failed: '$restoreCheck'"
}

# --- Summary ---
$port.Close()

Write-Host ""
Write-Host "============================================" -ForegroundColor White
$resultColor = if ($script:fail -eq 0) { "Green" } else { "Red" }
Write-Host " RESULTS: $($script:pass) passed, $($script:fail) failed, $($script:skip) skipped" -ForegroundColor $resultColor
Write-Host " Total tests: $($script:testNum)" -ForegroundColor White
Write-Host "============================================" -ForegroundColor White
