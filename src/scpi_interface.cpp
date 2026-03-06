/*
 * scpi_interface.cpp — SCPI command handlers, interface callbacks, and FreeRTOS task
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <cstring>
#include "appconfig.h"
#include "scpi_interface.h"
#include "RunningAverage.h"
#include <PRDC_AD7193.h>

/* ------------------------------------------------------------------ */
/*  Extern references to globals defined in PennerScale.cpp           */
/* ------------------------------------------------------------------ */
extern RunningAverage extADCRunAV;
extern float   extADCweight;
extern float   extADCweightMax;
extern int32_t extADCResult;
extern int32_t extADCResultCh0;
extern int32_t extADCResultCh1;
extern float   calValue;
extern int32_t zeroValue;
extern float   tareValue;
extern uint32_t calWeight;
extern e_unitVal calUnit;
extern e_unitVal unitVal;
extern float   vinVolts;
extern float   v5vVolts;
extern e_backlightEnable backlightEnable;
extern uint8_t backlightPWM;
extern volatile bool scpiEchoEnable;
extern volatile bool scpiPromptEnable;
extern const float kgtolbScalar;
extern const String unitAbbr[];
extern portMUX_TYPE measMux;
extern float   stabThreshold;
extern float   overloadCapacity;
extern bool    adaptiveFilterEnable;
extern float   adaptiveFilterPct;
extern uint32_t adaptiveFilterTimeUs;
extern bool    configSwitch1;
extern bool    configSwitch2;
extern PRDC_AD7193 AD7193;

/* ------------------------------------------------------------------ */
/*  SCPI interface callbacks                                          */
/* ------------------------------------------------------------------ */

static bool scpiResponseStart = true;

static size_t SCPI_Write(scpi_t *context, const char *data, size_t len) {
    (void) context;
    if (scpiPromptEnable && scpiResponseStart) {
        Serial.print("> ");
        scpiResponseStart = false;
    }
    return Serial.write(reinterpret_cast<const uint8_t *>(data), len);
}

static scpi_result_t SCPI_Flush(scpi_t *context) {
    (void) context;
    Serial.flush();
    scpiResponseStart = true;
    return SCPI_RES_OK;
}

static int SCPI_Error(scpi_t *context, int_fast16_t err) {
    (void) context;
    DBG_PRINTF("**ERROR: %d, \"%s\"\r\n", (int) err, SCPI_ErrorTranslate(err));
    return 0;
}

static scpi_result_t SCPI_Control(scpi_t *context, scpi_ctrl_name_t ctrl, scpi_reg_val_t val) {
    (void) context;
    (void) ctrl;
    (void) val;
    return SCPI_RES_OK;
}

static scpi_result_t SCPI_Reset(scpi_t *context) {
    (void) context;
    tareValue = 0.0f;
    extADCRunAV.clear();
    DBG_PRINTF("SCPI *RST executed\r\n");
    return SCPI_RES_OK;
}

static scpi_result_t My_CoreTstQ(scpi_t *context) {
    SCPI_ResultInt32(context, 0);
    return SCPI_RES_OK;
}

/* ------------------------------------------------------------------ */
/*  Unit choice list for CONFigure:UNIT                               */
/* ------------------------------------------------------------------ */
static const scpi_choice_def_t unit_choices[] = {
    {"KG", (int32_t) kg},
    {"LB", (int32_t) lb},
    SCPI_CHOICE_LIST_END
};

static const scpi_choice_def_t filter_choices[] = {
    {"SINC3", (int32_t) AD7193_MODE_SINC3},
    {"SINC4", (int32_t) AD7193_MODE_SINC4},
    SCPI_CHOICE_LIST_END
};

static inline uint8_t pwmPercentToDuty(uint8_t pct) {
    return (uint8_t)((uint16_t)pct * 255 / 100);
}

/* ------------------------------------------------------------------ */
/*  Measurement commands                                              */
/* ------------------------------------------------------------------ */

/* MEASure:WEIGht? — return averaged, unit-converted weight */
static scpi_result_t Meas_WeightQ(scpi_t *context) {
    taskENTER_CRITICAL(&measMux);
    float avg = extADCRunAV.getAverage();
    taskEXIT_CRITICAL(&measMux);
    SCPI_ResultFloat(context, avg);
    return SCPI_RES_OK;
}

/* MEASure:WEIGht:RAW? — combined raw ADC counts */
static scpi_result_t Meas_WeightRawQ(scpi_t *context) {
    SCPI_ResultInt32(context, extADCResult);
    return SCPI_RES_OK;
}

/* MEASure:WEIGht:RAW:CH0? */
static scpi_result_t Meas_WeightRawCh0Q(scpi_t *context) {
    SCPI_ResultInt32(context, extADCResultCh0);
    return SCPI_RES_OK;
}

/* MEASure:WEIGht:RAW:CH1? */
static scpi_result_t Meas_WeightRawCh1Q(scpi_t *context) {
    SCPI_ResultInt32(context, extADCResultCh1);
    return SCPI_RES_OK;
}

/* MEASure:WEIGht:MAX? */
static scpi_result_t Meas_WeightMaxQ(scpi_t *context) {
    taskENTER_CRITICAL(&measMux);
    float max = extADCweightMax;
    taskEXIT_CRITICAL(&measMux);
    SCPI_ResultFloat(context, max);
    return SCPI_RES_OK;
}

/* MEASure:WEIGht:MAX <float> — set/reset peak weight and persist */
static scpi_result_t Meas_WeightMax(scpi_t *context) {
    scpi_number_t val;
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    taskENTER_CRITICAL(&measMux);
    extADCweightMax = (float) val.content.value;
    taskEXIT_CRITICAL(&measMux);
    EEPROM.put(EEPROM_ADDR_WEIGHT_MAX, extADCweightMax);
    EEPROM.commit();
    return SCPI_RES_OK;
}

/* MEASure:WEIGht:AVERage:COUNt? — samples currently in running average buffer */
static scpi_result_t Meas_WeightAvgCountQ(scpi_t *context) {
    taskENTER_CRITICAL(&measMux);
    uint16_t cnt = extADCRunAV.getCount();
    taskEXIT_CRITICAL(&measMux);
    SCPI_ResultInt32(context, (int32_t) cnt);
    return SCPI_RES_OK;
}

/* MEASure:WEIGht:AVERage:SIZE? — running average buffer size */
static scpi_result_t Meas_WeightAvgSizeQ(scpi_t *context) {
    SCPI_ResultInt32(context, (int32_t) extADCRunAV.getSize());
    return SCPI_RES_OK;
}

/* MEASure:WEIGht:SDEViation? — running average standard deviation */
static scpi_result_t Meas_WeightSdevQ(scpi_t *context) {
    taskENTER_CRITICAL(&measMux);
    float sd = extADCRunAV.getStandardDeviation();
    taskEXIT_CRITICAL(&measMux);
    SCPI_ResultFloat(context, sd);
    return SCPI_RES_OK;
}

/* MEASure:WEIGht:STABle? — 1 if settled (sdev < threshold*capacity AND buffer full), 0 otherwise */
static scpi_result_t Meas_WeightStableQ(scpi_t *context) {
    taskENTER_CRITICAL(&measMux);
    float sd   = extADCRunAV.getStandardDeviation();
    bool  full = extADCRunAV.bufferIsFull();
    taskEXIT_CRITICAL(&measMux);
    SCPI_ResultBool(context, full && sd < (stabThreshold * overloadCapacity));
    return SCPI_RES_OK;
}

/* MEASure:WEIGht:GROSS? — weight before tare subtraction, in display units */
static scpi_result_t Meas_WeightGrossQ(scpi_t *context) {
    taskENTER_CRITICAL(&measMux);
    float avg = extADCRunAV.getAverage();
    taskEXIT_CRITICAL(&measMux);
    /* avg is already in display units (after tare subtraction).
       tareValue is in calUnit — convert to display units. */
    float tareDisplay = tareValue;
    if (calUnit == lb && unitVal == kg)       tareDisplay = tareValue / kgtolbScalar;
    else if (calUnit == kg && unitVal == lb)  tareDisplay = tareValue * kgtolbScalar;
    float gross = avg + tareDisplay;
    SCPI_ResultFloat(context, gross);
    return SCPI_RES_OK;
}

/* MEASure:WEIGht:OVERload? — 1 if abs(weight) > capacity, 0 otherwise */
static scpi_result_t Meas_WeightOverloadQ(scpi_t *context) {
    taskENTER_CRITICAL(&measMux);
    float avg = extADCRunAV.getAverage();
    taskEXIT_CRITICAL(&measMux);
    SCPI_ResultBool(context, fabsf(avg) > overloadCapacity);
    return SCPI_RES_OK;
}

/* ------------------------------------------------------------------ */
/*  Configuration commands                                            */
/* ------------------------------------------------------------------ */

/* CONFigure:UNIT <KG|LB> */
static scpi_result_t Conf_Unit(scpi_t *context) {
    int32_t val;
    if (!SCPI_ParamChoice(context, unit_choices, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    e_unitVal newUnit = (e_unitVal) val;
    if (newUnit != unitVal) {
        if (newUnit == lb) {
            overloadCapacity *= kgtolbScalar;
            extADCweightMax *= kgtolbScalar;
        } else {
            overloadCapacity /= kgtolbScalar;
            extADCweightMax /= kgtolbScalar;
        }
        extADCRunAV.clear();
        unitVal = newUnit;
    }
    return SCPI_RES_OK;
}

/* CONFigure:UNIT? */
static scpi_result_t Conf_UnitQ(scpi_t *context) {
    const char *name = NULL;
    SCPI_ChoiceToName(unit_choices, (int32_t) unitVal, &name);
    if (!name) name = "?";
    SCPI_ResultCharacters(context, name, strlen(name));
    return SCPI_RES_OK;
}

/* CONFigure:TARE [<value>] — no param = auto-tare, with param = set tare directly */
static scpi_result_t Conf_Tare(scpi_t *context) {
    scpi_number_t val;
    if (SCPI_ParamNumber(context, scpi_special_numbers_def, &val, FALSE)) {
        /* Explicit tare value provided */
        tareValue = (float) val.content.value;
    } else {
        /* No parameter — auto-tare from current reading */
        if (calValue == 0.0f) {
            SCPI_ErrorPush(context, SCPI_ERROR_EXECUTION_ERROR);
            return SCPI_RES_ERR;
        }
        tareValue = (extADCResult - zeroValue) / calValue;
    }
    extADCRunAV.clear();
    return SCPI_RES_OK;
}

/* CONFigure:TARE? — return current tare value */
static scpi_result_t Conf_TareQ(scpi_t *context) {
    SCPI_ResultFloat(context, tareValue);
    return SCPI_RES_OK;
}

/* CONFigure:ZERO — set current ADC reading as the zero reference and persist */
static scpi_result_t Conf_Zero(scpi_t *context) {
    (void) context;
    zeroValue = extADCResult;
    tareValue = 0;
    extADCRunAV.clear();
    EEPROM.put(EEPROM_ADDR_ZERO_VALUE, zeroValue);
    EEPROM.commit();
    return SCPI_RES_OK;
}

/* ------------------------------------------------------------------ */
/*  ADC configuration commands (runtime-only, not persisted)          */
/* ------------------------------------------------------------------ */

/* CONFigure:ADC:RATE <1-1023> */
static scpi_result_t Conf_AdcRate(scpi_t *context) {
    int32_t val;
    if (!SCPI_ParamInt32(context, &val, TRUE)) return SCPI_RES_ERR;
    if (val < 1 || val > 1023) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    AD7193.setRate((uint32_t) val);
    return SCPI_RES_OK;
}

/* CONFigure:ADC:RATE? */
static scpi_result_t Conf_AdcRateQ(scpi_t *context) {
    SCPI_ResultUInt32(context, AD7193.getRate());
    return SCPI_RES_OK;
}

/* CONFigure:ADC:FILTer <SINC3|SINC4> */
static scpi_result_t Conf_AdcFilter(scpi_t *context) {
    int32_t val;
    if (!SCPI_ParamChoice(context, filter_choices, &val, TRUE)) return SCPI_RES_ERR;
    AD7193.setFilterDeferred((uint32_t) val);
    return SCPI_RES_OK;
}

/* CONFigure:ADC:FILTer? */
static scpi_result_t Conf_AdcFilterQ(scpi_t *context) {
    const char *name = NULL;
    SCPI_ChoiceToName(filter_choices, (int32_t) AD7193.getFilter(), &name);
    if (!name) name = "?";
    SCPI_ResultCharacters(context, name, strlen(name));
    return SCPI_RES_OK;
}

/* CONFigure:ADC:NOTCh <ON|OFF> */
static scpi_result_t Conf_AdcNotch(scpi_t *context) {
    scpi_bool_t val;
    if (!SCPI_ParamBool(context, &val, TRUE)) return SCPI_RES_ERR;
    AD7193.setNotchFilterDeferred((bool) val);
    return SCPI_RES_OK;
}

/* CONFigure:ADC:NOTCh? */
static scpi_result_t Conf_AdcNotchQ(scpi_t *context) {
    SCPI_ResultBool(context, AD7193.getNotchFilter() == AD7193_MODE_REJ60);
    return SCPI_RES_OK;
}

/* CONFigure:STABility:THReshold <val> — set stability threshold (persists) */
static scpi_result_t Conf_StabThresh(scpi_t *context) {
    scpi_number_t val;
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    if (val.content.value <= 0.0) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    stabThreshold = (float) val.content.value;
    EEPROM.put(EEPROM_ADDR_STAB_THRESH, stabThreshold);
    EEPROM.commit();
    return SCPI_RES_OK;
}

/* CONFigure:STABility:THReshold? */
static scpi_result_t Conf_StabThreshQ(scpi_t *context) {
    SCPI_ResultFloat(context, stabThreshold);
    return SCPI_RES_OK;
}

/* CONFigure:OVERload:CAPacity <val> — set overload capacity (persists) */
static scpi_result_t Conf_OverCap(scpi_t *context) {
    scpi_number_t val;
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    if (val.content.value <= 0.0) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    overloadCapacity = (float) val.content.value;
    EEPROM.put(EEPROM_ADDR_OVER_CAP, overloadCapacity);
    EEPROM.put(EEPROM_ADDR_UNIT_VAL, unitVal);  // keep unit+capacity in sync
    EEPROM.commit();
    return SCPI_RES_OK;
}

/* CONFigure:OVERload:CAPacity? */
static scpi_result_t Conf_OverCapQ(scpi_t *context) {
    SCPI_ResultFloat(context, overloadCapacity);
    return SCPI_RES_OK;
}

/* CONFigure:FILTer:ADAPtive <ON|OFF> — enable/disable adaptive filter (persists) */
static scpi_result_t Conf_AdaptEnable(scpi_t *context) {
    scpi_bool_t val;
    if (!SCPI_ParamBool(context, &val, TRUE)) return SCPI_RES_ERR;
    adaptiveFilterEnable = (bool) val;
    EEPROM.put(EEPROM_ADDR_ADAPT_ENABLE, (uint8_t) adaptiveFilterEnable);
    EEPROM.commit();
    return SCPI_RES_OK;
}

/* CONFigure:FILTer:ADAPtive? */
static scpi_result_t Conf_AdaptEnableQ(scpi_t *context) {
    SCPI_ResultBool(context, adaptiveFilterEnable);
    return SCPI_RES_OK;
}

/* CONFigure:FILTer:ADAPtive:THReshold <val> — threshold as % of capacity (persists) */
static scpi_result_t Conf_AdaptThresh(scpi_t *context) {
    scpi_number_t val;
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    if (val.content.value <= 0.0) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    adaptiveFilterPct = (float) val.content.value;
    EEPROM.put(EEPROM_ADDR_ADAPT_THRESH, adaptiveFilterPct);
    EEPROM.commit();
    return SCPI_RES_OK;
}

/* CONFigure:FILTer:ADAPtive:THReshold? */
static scpi_result_t Conf_AdaptThreshQ(scpi_t *context) {
    SCPI_ResultFloat(context, adaptiveFilterPct);
    return SCPI_RES_OK;
}

/* CONFigure:FILTer:ADAPtive:TIME <val> — sustained deviation window in µs (persists) */
static scpi_result_t Conf_AdaptTime(scpi_t *context) {
    int32_t val;
    if (!SCPI_ParamInt32(context, &val, TRUE)) return SCPI_RES_ERR;
    if (val < 1 || val > 60000000) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    adaptiveFilterTimeUs = (uint32_t) val;
    EEPROM.put(EEPROM_ADDR_ADAPT_TIME, adaptiveFilterTimeUs);
    EEPROM.commit();
    return SCPI_RES_OK;
}

/* CONFigure:FILTer:ADAPtive:TIME? */
static scpi_result_t Conf_AdaptTimeQ(scpi_t *context) {
    SCPI_ResultInt32(context, (int32_t) adaptiveFilterTimeUs);
    return SCPI_RES_OK;
}

/* ------------------------------------------------------------------ */
/*  Calibration commands                                              */
/* ------------------------------------------------------------------ */

/* CALibration:VALue? — query calibration factor */
static scpi_result_t Cal_ValueQ(scpi_t *context) {
    SCPI_ResultFloat(context, calValue);
    return SCPI_RES_OK;
}

/* CALibration:VALue <float> — set calibration factor and persist (must be > 0) */
static scpi_result_t Cal_Value(scpi_t *context) {
    scpi_number_t val;
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    if (val.content.value <= 0.0) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    calValue = (float) val.content.value;
    EEPROM.put(EEPROM_ADDR_CAL_VALUE, calValue);
    EEPROM.commit();
    return SCPI_RES_OK;
}

/* CALibration:ZERO? — query zero-reference ADC value */
static scpi_result_t Cal_ZeroQ(scpi_t *context) {
    SCPI_ResultInt32(context, zeroValue);
    return SCPI_RES_OK;
}

/* CALibration:ZERO <int32> — set zero reference directly and persist */
static scpi_result_t Cal_Zero(scpi_t *context) {
    int32_t val;
    if (!SCPI_ParamInt32(context, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    zeroValue = val;
    EEPROM.put(EEPROM_ADDR_ZERO_VALUE, zeroValue);
    EEPROM.commit();
    return SCPI_RES_OK;
}

/* CALibration:WEIGht? — query calibration weight */
static scpi_result_t Cal_WeightQ(scpi_t *context) {
    SCPI_ResultUInt32(context, calWeight);
    return SCPI_RES_OK;
}

/* CALibration:WEIGht <uint32> — set calibration weight and persist (must be > 0) */
static scpi_result_t Cal_Weight(scpi_t *context) {
    int32_t val;
    if (!SCPI_ParamInt32(context, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    if (val <= 0) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    calWeight = (uint32_t) val;
    EEPROM.put(EEPROM_ADDR_CAL_WEIGHT, calWeight);
    EEPROM.commit();
    return SCPI_RES_OK;
}

/* CALibration:UNIT? — query calibration unit */
static scpi_result_t Cal_UnitQ(scpi_t *context) {
    const char *name = NULL;
    SCPI_ChoiceToName(unit_choices, (int32_t) calUnit, &name);
    if (!name) name = "?";
    SCPI_ResultCharacters(context, name, strlen(name));
    return SCPI_RES_OK;
}

/* CALibration:UNIT <KG|LB> — set calibration unit and persist */
static scpi_result_t Cal_Unit(scpi_t *context) {
    int32_t val;
    if (!SCPI_ParamChoice(context, unit_choices, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    calUnit = (e_unitVal) val;
    EEPROM.put(EEPROM_ADDR_CAL_UNIT, calUnit);
    EEPROM.commit();
    return SCPI_RES_OK;
}

/* ------------------------------------------------------------------ */
/*  Remote calibration commands (blocking)                            */
/* ------------------------------------------------------------------ */

/* Helper: clear running average and block until buffer is full (settled).
   Returns true on success, false on timeout (30 s). */
static bool waitForSettle(void) {
    taskENTER_CRITICAL(&measMux);
    extADCRunAV.clear();
    taskEXIT_CRITICAL(&measMux);

    const TickType_t timeout = pdMS_TO_TICKS(30000);
    TickType_t start = xTaskGetTickCount();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        taskENTER_CRITICAL(&measMux);
        bool full = extADCRunAV.bufferIsFull();
        taskEXIT_CRITICAL(&measMux);
        if (full) return true;
        if ((xTaskGetTickCount() - start) >= timeout) return false;
    }
}

/* CALibration:ZERO:EXEC — block until settled, capture zero, persist */
static scpi_result_t Cal_ZeroExec(scpi_t *context) {
    if (!waitForSettle()) {
        SCPI_ErrorPush(context, SCPI_ERROR_EXECUTION_ERROR);
        return SCPI_RES_ERR;
    }
    zeroValue = extADCResult;
    tareValue = 0;
    EEPROM.put(EEPROM_ADDR_ZERO_VALUE, zeroValue);
    EEPROM.commit();
    taskENTER_CRITICAL(&measMux);
    extADCRunAV.clear();
    taskEXIT_CRITICAL(&measMux);
    DBG_PRINTF("CAL:ZERO:EXEC zeroValue=%ld\r\n", (long) zeroValue);
    return SCPI_RES_OK;
}

/* CALibration:SPAN:EXEC — block until settled, compute calValue, persist */
static scpi_result_t Cal_SpanExec(scpi_t *context) {
    if (calWeight == 0) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    if (!waitForSettle()) {
        SCPI_ErrorPush(context, SCPI_ERROR_EXECUTION_ERROR);
        return SCPI_RES_ERR;
    }
    float newCalValue = (float)(extADCResult - zeroValue) / (float) calWeight;
    if (newCalValue <= 0.0f) {
        SCPI_ErrorPush(context, SCPI_ERROR_EXECUTION_ERROR);
        return SCPI_RES_ERR;
    }
    calValue = newCalValue;
    EEPROM.put(EEPROM_ADDR_CAL_VALUE, calValue);
    EEPROM.put(EEPROM_ADDR_ZERO_VALUE, zeroValue);
    EEPROM.put(EEPROM_ADDR_CAL_WEIGHT, calWeight);
    EEPROM.put(EEPROM_ADDR_CAL_UNIT, calUnit);
    EEPROM.commit();
    taskENTER_CRITICAL(&measMux);
    extADCRunAV.clear();
    taskEXIT_CRITICAL(&measMux);
    DBG_PRINTF("CAL:SPAN:EXEC calValue=%.6f\r\n", (double) calValue);
    return SCPI_RES_OK;
}

/* ------------------------------------------------------------------ */
/*  System commands                                                   */
/* ------------------------------------------------------------------ */

/* SYSTem:BACKlight <ON|OFF> (bool) */
static scpi_result_t Sys_Backlight(scpi_t *context) {
    scpi_bool_t val;
    if (!SCPI_ParamBool(context, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    backlightEnable = val ? on : off;
    ledcWrite(LCD_BACKLIGHT, pwmPercentToDuty(backlightPWM) * (backlightEnable != off));
    return SCPI_RES_OK;
}

/* SYSTem:BACKlight? */
static scpi_result_t Sys_BacklightQ(scpi_t *context) {
    SCPI_ResultBool(context, backlightEnable != off);
    return SCPI_RES_OK;
}

/* SYSTem:BACKlight:PWM <0-100> — set backlight PWM duty cycle (percent) */
static scpi_result_t Sys_BacklightPWM(scpi_t *context) {
    int32_t val;
    if (!SCPI_ParamInt32(context, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    if (val < 0 || val > 100) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    backlightPWM = (uint8_t) val;
    if (backlightEnable != off) {
        ledcWrite(LCD_BACKLIGHT, pwmPercentToDuty(backlightPWM));
    }
    EEPROM.put(EEPROM_ADDR_BACKLIGHT_PWM, backlightPWM);
    EEPROM.commit();
    return SCPI_RES_OK;
}

/* SYSTem:BACKlight:PWM? */
static scpi_result_t Sys_BacklightPWMQ(scpi_t *context) {
    SCPI_ResultInt32(context, (int32_t) backlightPWM);
    return SCPI_RES_OK;
}

/* SYSTem:ECHO <ON|OFF> */
static scpi_result_t Sys_Echo(scpi_t *context) {
    scpi_bool_t val;
    if (!SCPI_ParamBool(context, &val, TRUE)) return SCPI_RES_ERR;
    scpiEchoEnable = (bool) val;
    EEPROM.put(EEPROM_ADDR_ECHO, (uint8_t) scpiEchoEnable);
    EEPROM.commit();
    return SCPI_RES_OK;
}

/* SYSTem:ECHO? */
static scpi_result_t Sys_EchoQ(scpi_t *context) {
    SCPI_ResultBool(context, scpiEchoEnable);
    return SCPI_RES_OK;
}

/* SYSTem:PROMpt <ON|OFF> */
static scpi_result_t Sys_Prompt(scpi_t *context) {
    scpi_bool_t val;
    if (!SCPI_ParamBool(context, &val, TRUE)) return SCPI_RES_ERR;
    scpiPromptEnable = (bool) val;
    EEPROM.put(EEPROM_ADDR_PROMPT, (uint8_t) scpiPromptEnable);
    EEPROM.commit();
    return SCPI_RES_OK;
}

/* SYSTem:PROMpt? */
static scpi_result_t Sys_PromptQ(scpi_t *context) {
    SCPI_ResultBool(context, scpiPromptEnable);
    return SCPI_RES_OK;
}

/* SYSTem:POWer:VOLTage:BATTery? */
static scpi_result_t Sys_VbattQ(scpi_t *context) {
    SCPI_ResultFloat(context, vinVolts);
    return SCPI_RES_OK;
}

/* SYSTem:POWer:VOLTage:SUPPly? */
static scpi_result_t Sys_VsuppQ(scpi_t *context) {
    SCPI_ResultFloat(context, v5vVolts);
    return SCPI_RES_OK;
}

/* SYSTem:POWer:GOOD:VDD? — 3.3V power good */
static scpi_result_t Sys_PG3V3Q(scpi_t *context) {
    SCPI_ResultBool(context, digitalRead(V3V3_PG));
    return SCPI_RES_OK;
}

/* SYSTem:POWer:GOOD:V5A? — 5V power good */
static scpi_result_t Sys_PG5VQ(scpi_t *context) {
    SCPI_ResultBool(context, digitalRead(V5_A_PG));
    return SCPI_RES_OK;
}

/* SYSTem:CONFig:SW1? — DIP switch 1 state */
static scpi_result_t Sys_ConfSw1Q(scpi_t *context) {
    SCPI_ResultBool(context, configSwitch1);
    return SCPI_RES_OK;
}

/* SYSTem:CONFig:SW2? — DIP switch 2 state */
static scpi_result_t Sys_ConfSw2Q(scpi_t *context) {
    SCPI_ResultBool(context, configSwitch2);
    return SCPI_RES_OK;
}

/* SYSTem:FW? — firmware version string */
static scpi_result_t Sys_FwQ(scpi_t *context) {
    SCPI_ResultCharacters(context, FW_VER, strlen(FW_VER));
    return SCPI_RES_OK;
}

/* Forward declarations for functions in PennerScale.cpp */
extern void powerDown(void);
extern void wakeFromIdleMode(void);
static scpi_result_t Sys_PowerDown(scpi_t *context) {
    (void) context;
    powerDown();
    return SCPI_RES_OK;  /* may not be reached */
}

/* SYSTem:EEPROM? — dump all persisted EEPROM values */
static scpi_result_t Sys_EepromQ(scpi_t *context) {
    float            ee_calValue;   EEPROM.get(EEPROM_ADDR_CAL_VALUE,  ee_calValue);
    int32_t          ee_zeroValue;  EEPROM.get(EEPROM_ADDR_ZERO_VALUE, ee_zeroValue);
    e_backlightEnable ee_backlight; EEPROM.get(EEPROM_ADDR_BACKLIGHT,  ee_backlight);
    e_unitVal        ee_unit;       EEPROM.get(EEPROM_ADDR_UNIT_VAL,   ee_unit);
    uint32_t         ee_calWeight;  EEPROM.get(EEPROM_ADDR_CAL_WEIGHT, ee_calWeight);
    e_unitVal        ee_calUnit;    EEPROM.get(EEPROM_ADDR_CAL_UNIT,   ee_calUnit);
    float            ee_weightMax;  EEPROM.get(EEPROM_ADDR_WEIGHT_MAX, ee_weightMax);
    uint8_t          ee_backlightPWM; EEPROM.get(EEPROM_ADDR_BACKLIGHT_PWM, ee_backlightPWM);
    uint8_t          ee_echo;   EEPROM.get(EEPROM_ADDR_ECHO,   ee_echo);
    uint8_t          ee_prompt; EEPROM.get(EEPROM_ADDR_PROMPT,  ee_prompt);
    float            ee_stabThresh; EEPROM.get(EEPROM_ADDR_STAB_THRESH, ee_stabThresh);
    float            ee_overCap;    EEPROM.get(EEPROM_ADDR_OVER_CAP,    ee_overCap);
    uint8_t          ee_adaptEn;   EEPROM.get(EEPROM_ADDR_ADAPT_ENABLE, ee_adaptEn);
    float            ee_adaptThr;  EEPROM.get(EEPROM_ADDR_ADAPT_THRESH, ee_adaptThr);
    uint32_t         ee_adaptTime; EEPROM.get(EEPROM_ADDR_ADAPT_TIME,   ee_adaptTime);

    const char *unit_name = NULL, *cal_unit_name = NULL;
    SCPI_ChoiceToName(unit_choices, (int32_t) ee_unit, &unit_name);
    SCPI_ChoiceToName(unit_choices, (int32_t) ee_calUnit, &cal_unit_name);

    char buf[384];
    snprintf(buf, sizeof(buf),
        "calValue=%.6f,zeroValue=%ld,backlight=%d,unit=%s,"
        "calWeight=%lu,calUnit=%s,weightMax=%.4f,backlightPWM=%u,"
        "echo=%u,prompt=%u,stabThresh=%.4f,overCap=%.4f,"
        "adaptEn=%u,adaptThr=%.4f,adaptTimeUs=%lu",
        (double) ee_calValue, (long) ee_zeroValue, (int) ee_backlight,
        unit_name ? unit_name : "?",
        (unsigned long) ee_calWeight, cal_unit_name ? cal_unit_name : "?",
        (double) ee_weightMax, (unsigned) ee_backlightPWM,
        (unsigned) ee_echo, (unsigned) ee_prompt,
        (double) ee_stabThresh, (double) ee_overCap,
        (unsigned) ee_adaptEn, (double) ee_adaptThr, (unsigned long) ee_adaptTime);
    SCPI_ResultCharacters(context, buf, strlen(buf));
    return SCPI_RES_OK;
}

/* SYSTem:EEPROM:COMMit — flush any pending EEPROM writes to flash */
static scpi_result_t Sys_EepromCommit(scpi_t *context) {
    if (!EEPROM.commit()) {
        SCPI_ErrorPush(context, SCPI_ERROR_SYSTEM_ERROR);
        return SCPI_RES_ERR;
    }
    return SCPI_RES_OK;
}

/* ------------------------------------------------------------------ */
/*  FreeRTOS diagnostic commands                                      */
/* ------------------------------------------------------------------ */
#if FREERTOS_DIAG

/* SYSTem:DIAGnostic:STATS? — CPU run-time percentage per task */
static scpi_result_t Sys_DiagStatsQ(scpi_t *context) {
    static char buf[512];
    vTaskGetRunTimeStats(buf);
    SCPI_ResultCharacters(context, buf, strlen(buf));
    return SCPI_RES_OK;
}

/* SYSTem:DIAGnostic:LIST? — task state, priority, stack watermark */
static scpi_result_t Sys_DiagListQ(scpi_t *context) {
    static char buf[512];
    vTaskList(buf);
    SCPI_ResultCharacters(context, buf, strlen(buf));
    return SCPI_RES_OK;
}

#endif // FREERTOS_DIAG

/* ------------------------------------------------------------------ */
/*  Debug log commands                                                */
/* ------------------------------------------------------------------ */
#if SCPI_DEBUG

/* SYSTem:LOG? — read debug log ring buffer */
static scpi_result_t Sys_LogQ(scpi_t *context) {
    static char buf[DBG_LOG_BUF_SIZE];
    size_t n = dbg_read(buf, sizeof(buf));
    if (n > 0) {
        SCPI_ResultCharacters(context, buf, n);
    } else {
        SCPI_ResultCharacters(context, "(empty)", 7);
    }
    return SCPI_RES_OK;
}

/* SYSTem:LOG:CLEar — clear the debug log */
static scpi_result_t Sys_LogClear(scpi_t *context) {
    (void) context;
    dbg_clear();
    return SCPI_RES_OK;
}

#endif // SCPI_DEBUG

/* ------------------------------------------------------------------ */
/*  Command table                                                     */
/* ------------------------------------------------------------------ */

static const scpi_command_t scpi_commands[] = {
    /* IEEE 488.2 mandatory */
    { .pattern = "*CLS",  .callback = SCPI_CoreCls, },
    { .pattern = "*ESE",  .callback = SCPI_CoreEse, },
    { .pattern = "*ESE?", .callback = SCPI_CoreEseQ, },
    { .pattern = "*ESR?", .callback = SCPI_CoreEsrQ, },
    { .pattern = "*IDN?", .callback = SCPI_CoreIdnQ, },
    { .pattern = "*OPC",  .callback = SCPI_CoreOpc, },
    { .pattern = "*OPC?", .callback = SCPI_CoreOpcQ, },
    { .pattern = "*RST",  .callback = SCPI_CoreRst, },
    { .pattern = "*SRE",  .callback = SCPI_CoreSre, },
    { .pattern = "*SRE?", .callback = SCPI_CoreSreQ, },
    { .pattern = "*STB?", .callback = SCPI_CoreStbQ, },
    { .pattern = "*TST?", .callback = My_CoreTstQ, },
    { .pattern = "*WAI",  .callback = SCPI_CoreWai, },

    /* Required SCPI commands */
    { .pattern = "SYSTem:ERRor[:NEXT]?",           .callback = SCPI_SystemErrorNextQ, },
    { .pattern = "SYSTem:ERRor:COUNt?",            .callback = SCPI_SystemErrorCountQ, },
    { .pattern = "SYSTem:VERSion?",                .callback = SCPI_SystemVersionQ, },
    { .pattern = "STATus:QUEStionable[:EVENt]?",   .callback = SCPI_StatusQuestionableEventQ, },
    { .pattern = "STATus:QUEStionable:ENABle",     .callback = SCPI_StatusQuestionableEnable, },
    { .pattern = "STATus:QUEStionable:ENABle?",    .callback = SCPI_StatusQuestionableEnableQ, },
    { .pattern = "STATus:PRESet",                   .callback = SCPI_StatusPreset, },

    /* Measurement */
    { .pattern = "MEASure:WEIGht?",          .callback = Meas_WeightQ, },
    { .pattern = "MEASure:WEIGht:RAW?",      .callback = Meas_WeightRawQ, },
    { .pattern = "MEASure:WEIGht:RAW:CH0?",  .callback = Meas_WeightRawCh0Q, },
    { .pattern = "MEASure:WEIGht:RAW:CH1?",  .callback = Meas_WeightRawCh1Q, },
    { .pattern = "MEASure:WEIGht:MAX",        .callback = Meas_WeightMax, },
    { .pattern = "MEASure:WEIGht:MAX?",       .callback = Meas_WeightMaxQ, },
    { .pattern = "MEASure:WEIGht:AVERage:COUNt?", .callback = Meas_WeightAvgCountQ, },
    { .pattern = "MEASure:WEIGht:AVERage:SIZE?",  .callback = Meas_WeightAvgSizeQ, },
    { .pattern = "MEASure:WEIGht:SDEViation?",    .callback = Meas_WeightSdevQ, },
    { .pattern = "MEASure:WEIGht:STABle?",        .callback = Meas_WeightStableQ, },
    { .pattern = "MEASure:WEIGht:GROSS?",         .callback = Meas_WeightGrossQ, },
    { .pattern = "MEASure:WEIGht:OVERload?",      .callback = Meas_WeightOverloadQ, },

    /* Configuration */
    { .pattern = "CONFigure:UNIT",   .callback = Conf_Unit, },
    { .pattern = "CONFigure:UNIT?",  .callback = Conf_UnitQ, },
    { .pattern = "CONFigure:TARE",   .callback = Conf_Tare, },
    { .pattern = "CONFigure:TARE?",  .callback = Conf_TareQ, },
    { .pattern = "CONFigure:ZERO",   .callback = Conf_Zero, },
    { .pattern = "CONFigure:ADC:RATE",          .callback = Conf_AdcRate, },
    { .pattern = "CONFigure:ADC:RATE?",         .callback = Conf_AdcRateQ, },
    { .pattern = "CONFigure:ADC:FILTer",        .callback = Conf_AdcFilter, },
    { .pattern = "CONFigure:ADC:FILTer?",       .callback = Conf_AdcFilterQ, },
    { .pattern = "CONFigure:ADC:NOTCh",         .callback = Conf_AdcNotch, },
    { .pattern = "CONFigure:ADC:NOTCh?",        .callback = Conf_AdcNotchQ, },
    { .pattern = "CONFigure:STABility:THReshold",  .callback = Conf_StabThresh, },
    { .pattern = "CONFigure:STABility:THReshold?", .callback = Conf_StabThreshQ, },
    { .pattern = "CONFigure:OVERload:CAPacity",    .callback = Conf_OverCap, },
    { .pattern = "CONFigure:OVERload:CAPacity?",   .callback = Conf_OverCapQ, },
    { .pattern = "CONFigure:FILTer:ADAPtive",             .callback = Conf_AdaptEnable, },
    { .pattern = "CONFigure:FILTer:ADAPtive?",            .callback = Conf_AdaptEnableQ, },
    { .pattern = "CONFigure:FILTer:ADAPtive:THReshold",   .callback = Conf_AdaptThresh, },
    { .pattern = "CONFigure:FILTer:ADAPtive:THReshold?",  .callback = Conf_AdaptThreshQ, },
    { .pattern = "CONFigure:FILTer:ADAPtive:TIME",        .callback = Conf_AdaptTime, },
    { .pattern = "CONFigure:FILTer:ADAPtive:TIME?",       .callback = Conf_AdaptTimeQ, },

    /* Calibration */
    { .pattern = "CALibration:VALue",   .callback = Cal_Value, },
    { .pattern = "CALibration:VALue?",  .callback = Cal_ValueQ, },
    { .pattern = "CALibration:ZERO",    .callback = Cal_Zero, },
    { .pattern = "CALibration:ZERO?",   .callback = Cal_ZeroQ, },
    { .pattern = "CALibration:WEIGht",  .callback = Cal_Weight, },
    { .pattern = "CALibration:WEIGht?", .callback = Cal_WeightQ, },
    { .pattern = "CALibration:UNIT",    .callback = Cal_Unit, },
    { .pattern = "CALibration:UNIT?",   .callback = Cal_UnitQ, },
    { .pattern = "CALibration:ZERO:EXEC", .callback = Cal_ZeroExec, },
    { .pattern = "CALibration:SPAN:EXEC", .callback = Cal_SpanExec, },

    /* System */
    { .pattern = "SYSTem:BACKlight",              .callback = Sys_Backlight, },
    { .pattern = "SYSTem:BACKlight?",             .callback = Sys_BacklightQ, },
    { .pattern = "SYSTem:BACKlight:PWM",          .callback = Sys_BacklightPWM, },
    { .pattern = "SYSTem:BACKlight:PWM?",         .callback = Sys_BacklightPWMQ, },
    { .pattern = "SYSTem:POWer:VOLTage:BATTery?", .callback = Sys_VbattQ, },
    { .pattern = "SYSTem:POWer:VOLTage:SUPPly?",  .callback = Sys_VsuppQ, },
    { .pattern = "SYSTem:POWer:GOOD:VDD?",        .callback = Sys_PG3V3Q, },
    { .pattern = "SYSTem:POWer:GOOD:V5A?",        .callback = Sys_PG5VQ, },
    { .pattern = "SYSTem:POWer:DOWN",              .callback = Sys_PowerDown, },
    { .pattern = "SYSTem:CONFig:SW1?",             .callback = Sys_ConfSw1Q, },
    { .pattern = "SYSTem:CONFig:SW2?",             .callback = Sys_ConfSw2Q, },
    { .pattern = "SYSTem:FW?",                     .callback = Sys_FwQ, },
    { .pattern = "SYSTem:ECHO",                    .callback = Sys_Echo, },
    { .pattern = "SYSTem:ECHO?",                   .callback = Sys_EchoQ, },
    { .pattern = "SYSTem:PROMpt",                  .callback = Sys_Prompt, },
    { .pattern = "SYSTem:PROMpt?",                 .callback = Sys_PromptQ, },
    { .pattern = "SYSTem:EEPROM?",                 .callback = Sys_EepromQ, },
    { .pattern = "SYSTem:EEPROM:COMMit",           .callback = Sys_EepromCommit, },

#if FREERTOS_DIAG
    /* Diagnostics */
    { .pattern = "SYSTem:DIAGnostic:STATS?",     .callback = Sys_DiagStatsQ, },
    { .pattern = "SYSTem:DIAGnostic:LIST?",       .callback = Sys_DiagListQ, },
#endif

#if SCPI_DEBUG
    /* Debug log */
    { .pattern = "SYSTem:LOG?",                   .callback = Sys_LogQ, },
    { .pattern = "SYSTem:LOG:CLEar",              .callback = Sys_LogClear, },
#endif

    SCPI_CMD_LIST_END
};

/* ------------------------------------------------------------------ */
/*  SCPI interface struct & buffers                                   */
/* ------------------------------------------------------------------ */

static scpi_interface_t scpi_interface = {
    .error   = SCPI_Error,
    .write   = SCPI_Write,
    .control = SCPI_Control,
    .flush   = SCPI_Flush,
    .reset   = SCPI_Reset,
};

static char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];
static scpi_error_t scpi_error_queue_data[SCPI_ERROR_QUEUE_SIZE];

scpi_t scpi_context;

/* ------------------------------------------------------------------ */
/*  FreeRTOS task                                                     */
/* ------------------------------------------------------------------ */

void TaskSCPI(void *pvParameters) {
    (void) pvParameters;

    SCPI_Init(&scpi_context,
              scpi_commands,
              &scpi_interface,
              scpi_units_def,
              SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4,
              scpi_input_buffer, sizeof(scpi_input_buffer),
              scpi_error_queue_data, SCPI_ERROR_QUEUE_SIZE);

    DBG_PRINTF("SCPI parser initialized.\r\n");

    if (scpiPromptEnable) {
        Serial.print("> ");
    }

    uint8_t buf[64];
    for (;;) {
        // Drain all available bytes before sleeping to avoid FIFO overflow
        // (at 115200 baud, ~576 bytes can arrive during a 50ms sleep)
        if (Serial.available() > 0) {
            wakeFromIdleMode();
        }
        while (Serial.available() > 0) {
            int avail = Serial.available();
            int toRead = (avail > (int) sizeof(buf)) ? (int) sizeof(buf) : avail;
            int n = Serial.readBytes(buf, toRead);
            if (n > 0) {
                if (scpiEchoEnable) {
                    for (int i = 0; i < n; i++) {
                        if (buf[i] == '\r') {
                            Serial.print("\r\n");
                            if (i + 1 < n && buf[i + 1] == '\n') i++;
                        } else if (buf[i] == '\n') {
                            Serial.print("\r\n");
                        } else {
                            Serial.write(buf[i]);
                        }
                    }
                }
                SCPI_Input(&scpi_context, (const char *) buf, n);
                if (scpiPromptEnable &&
                    (memchr(buf, '\n', n) || memchr(buf, '\r', n))) {
                    Serial.print("> ");
                }
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
