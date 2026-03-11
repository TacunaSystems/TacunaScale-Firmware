/*
 * scpi_interface.cpp — SCPI command handlers, interface callbacks, and FreeRTOS task
 *
 * All measurement, configuration, and calibration commands are per-channel
 * (CH0 / CH1).  Measurement also provides :SUM? variants that combine both
 * channels, converting CH1's value into CH0's display unit.
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <cstring>
#include <cfloat>
#include "appconfig.h"
#include "scpi_interface.h"
#include "RunningAverage.h"
#include <PRDC_AD7193.h>

/* ------------------------------------------------------------------ */
/*  Extern references to globals defined in PennerScale.cpp           */
/* ------------------------------------------------------------------ */
extern RunningAverage extADCRunAV[NUM_CHANNELS];
extern float   extADCweight[NUM_CHANNELS];
extern float   extADCweightMax[NUM_CHANNELS];
extern int32_t extADCResult;
extern int32_t extADCResultCh[NUM_CHANNELS];
extern float   calValue[NUM_CHANNELS];
extern int32_t zeroValue[NUM_CHANNELS];
extern float   tareValue[NUM_CHANNELS];
extern uint32_t calWeight[NUM_CHANNELS];
extern e_unitVal calUnit[NUM_CHANNELS];
extern e_unitVal unitVal[NUM_CHANNELS];
extern float   vinVolts;
extern float   v5vVolts;
extern e_backlightEnable backlightEnable;
extern uint8_t backlightPWM;
extern volatile bool scpiEchoEnable;
extern volatile bool scpiPromptEnable;
extern const float kgtolbScalar;
extern const float kgtoNScalar;
extern const float NmtolbftScalar;
extern const char* const unitAbbr[];
extern float unitConversionFactor(e_unitVal from, e_unitVal to);
extern portMUX_TYPE measMux;
extern float   stabThreshold[NUM_CHANNELS];
extern float   overloadCapacity[NUM_CHANNELS];
extern bool    adaptiveFilterEnable[NUM_CHANNELS];
extern float   adaptiveFilterPct[NUM_CHANNELS];
extern uint32_t adaptiveFilterTimeUs[NUM_CHANNELS];
extern bool    configSwitch1;
extern bool    adcInvert[NUM_CHANNELS];
extern bool    configSwitch2;
extern e_displayMode displayMode;
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
    for (int c = 0; c < NUM_CHANNELS; c++) {
        tareValue[c] = 0.0f;
        extADCRunAV[c].clear();
    }
    DBG_PRINTF("SCPI *RST executed\r\n");
    return SCPI_RES_OK;
}

static scpi_result_t My_CoreTstQ(scpi_t *context) {
    SCPI_ResultInt32(context, 0);
    return SCPI_RES_OK;
}

/* ------------------------------------------------------------------ */
/*  Choice lists                                                      */
/* ------------------------------------------------------------------ */
static const scpi_choice_def_t unit_choices[] = {
    {"KG",   (int32_t) kg},
    {"LB",   (int32_t) lb},
    {"N",    (int32_t) N},
    {"NM",   (int32_t) Nm},
    {"LBFT", (int32_t) lbft},
    SCPI_CHOICE_LIST_END
};

static const scpi_choice_def_t filter_choices[] = {
    {"SINC3", (int32_t) AD7193_MODE_SINC3},
    {"SINC4", (int32_t) AD7193_MODE_SINC4},
    SCPI_CHOICE_LIST_END
};

static const scpi_choice_def_t display_mode_choices[] = {
    {"SINGle",  (int32_t) DISP_SINGLE},
    {"DUAL",    (int32_t) DISP_DUAL},
    {"SUM",     (int32_t) DISP_SUM},
    SCPI_CHOICE_LIST_END
};

static inline uint8_t pwmPercentToDuty(uint8_t pct) {
    return (uint8_t)((uint16_t)pct * 255 / 100);
}

/* ------------------------------------------------------------------ */
/*  EEPROM address helpers — map channel index to EEPROM address      */
/* ------------------------------------------------------------------ */
static int eepromAddrCalValue(int ch)  { return ch ? EEPROM_ADDR_CH1_CAL_VALUE  : EEPROM_ADDR_CAL_VALUE; }
static int eepromAddrZeroValue(int ch) { return ch ? EEPROM_ADDR_CH1_ZERO_VALUE : EEPROM_ADDR_ZERO_VALUE; }
static int eepromAddrUnitVal(int ch)   { return ch ? EEPROM_ADDR_CH1_UNIT_VAL   : EEPROM_ADDR_UNIT_VAL; }
static int eepromAddrCalWeight(int ch) { return ch ? EEPROM_ADDR_CH1_CAL_WEIGHT : EEPROM_ADDR_CAL_WEIGHT; }
static int eepromAddrCalUnit(int ch)   { return ch ? EEPROM_ADDR_CH1_CAL_UNIT   : EEPROM_ADDR_CAL_UNIT; }
static int eepromAddrWeightMax(int ch) { return ch ? EEPROM_ADDR_CH1_WEIGHT_MAX : EEPROM_ADDR_WEIGHT_MAX; }
static int eepromAddrOverCap(int ch)   { return ch ? EEPROM_ADDR_CH1_OVER_CAP   : EEPROM_ADDR_OVER_CAP; }
static int eepromAddrStabThresh(int ch)   { return ch ? EEPROM_ADDR_CH1_STAB_THRESH  : EEPROM_ADDR_STAB_THRESH; }
static int eepromAddrAdaptEnable(int ch)  { return ch ? EEPROM_ADDR_CH1_ADAPT_ENABLE : EEPROM_ADDR_ADAPT_ENABLE; }
static int eepromAddrAdaptThresh(int ch)  { return ch ? EEPROM_ADDR_CH1_ADAPT_THRESH : EEPROM_ADDR_ADAPT_THRESH; }
static int eepromAddrAdaptTime(int ch)    { return ch ? EEPROM_ADDR_CH1_ADAPT_TIME   : EEPROM_ADDR_ADAPT_TIME; }
static int eepromAddrAdcInvert(int ch)    { return ch ? EEPROM_ADDR_CH1_ADC_INVERT   : EEPROM_ADDR_ADC_INVERT; }

/* ------------------------------------------------------------------ */
/*  Measurement commands — per-channel + SUM                          */
/* ------------------------------------------------------------------ */

/* --- Weight (averaged, unit-converted) --- */

static scpi_result_t Meas_WeightChQ(scpi_t *context, int ch) {
    taskENTER_CRITICAL(&measMux);
    float avg = extADCRunAV[ch].getAverage();
    taskEXIT_CRITICAL(&measMux);
    SCPI_ResultFloat(context, avg);
    return SCPI_RES_OK;
}
static scpi_result_t Meas_WeightCh0Q(scpi_t *context) { return Meas_WeightChQ(context, 0); }
static scpi_result_t Meas_WeightCh1Q(scpi_t *context) { return Meas_WeightChQ(context, 1); }

/* Sum: both channels converted to CH0's display unit */
static scpi_result_t Meas_WeightSumQ(scpi_t *context) {
    taskENTER_CRITICAL(&measMux);
    float avg0 = extADCRunAV[0].getAverage();
    float avg1 = extADCRunAV[1].getAverage();
    taskEXIT_CRITICAL(&measMux);
    float sum = avg0 + avg1 * unitConversionFactor(unitVal[1], unitVal[0]);
    SCPI_ResultFloat(context, sum);
    return SCPI_RES_OK;
}

/* --- Raw ADC counts --- */

static scpi_result_t Meas_WeightRawChQ(scpi_t *context, int ch) {
    SCPI_ResultInt32(context, extADCResultCh[ch]);
    return SCPI_RES_OK;
}
static scpi_result_t Meas_WeightRawCh0Q(scpi_t *context) { return Meas_WeightRawChQ(context, 0); }
static scpi_result_t Meas_WeightRawCh1Q(scpi_t *context) { return Meas_WeightRawChQ(context, 1); }

/* Combined raw (both channels summed) */
static scpi_result_t Meas_WeightRawSumQ(scpi_t *context) {
    SCPI_ResultInt32(context, extADCResult);
    return SCPI_RES_OK;
}

/* --- Peak / Max weight --- */

static scpi_result_t Meas_WeightMaxChQ(scpi_t *context, int ch) {
    taskENTER_CRITICAL(&measMux);
    float mx = extADCweightMax[ch];
    taskEXIT_CRITICAL(&measMux);
    SCPI_ResultFloat(context, mx);
    return SCPI_RES_OK;
}
static scpi_result_t Meas_WeightMaxCh0Q(scpi_t *context) { return Meas_WeightMaxChQ(context, 0); }
static scpi_result_t Meas_WeightMaxCh1Q(scpi_t *context) { return Meas_WeightMaxChQ(context, 1); }

static scpi_result_t Meas_WeightMaxCh(scpi_t *context, int ch) {
    scpi_number_t val;
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    taskENTER_CRITICAL(&measMux);
    extADCweightMax[ch] = (float) val.content.value;
    taskEXIT_CRITICAL(&measMux);
    EEPROM.put(eepromAddrWeightMax(ch), extADCweightMax[ch]);
    EEPROM.commit();
    return SCPI_RES_OK;
}
static scpi_result_t Meas_WeightMaxCh0(scpi_t *context) { return Meas_WeightMaxCh(context, 0); }
static scpi_result_t Meas_WeightMaxCh1(scpi_t *context) { return Meas_WeightMaxCh(context, 1); }

/* --- Average count --- */

static scpi_result_t Meas_WeightAvgCountChQ(scpi_t *context, int ch) {
    taskENTER_CRITICAL(&measMux);
    uint16_t cnt = extADCRunAV[ch].getCount();
    taskEXIT_CRITICAL(&measMux);
    SCPI_ResultInt32(context, (int32_t) cnt);
    return SCPI_RES_OK;
}
static scpi_result_t Meas_WeightAvgCountCh0Q(scpi_t *context) { return Meas_WeightAvgCountChQ(context, 0); }
static scpi_result_t Meas_WeightAvgCountCh1Q(scpi_t *context) { return Meas_WeightAvgCountChQ(context, 1); }

/* --- Average size --- */

static scpi_result_t Meas_WeightAvgSizeChQ(scpi_t *context, int ch) {
    SCPI_ResultInt32(context, (int32_t) extADCRunAV[ch].getSize());
    return SCPI_RES_OK;
}
static scpi_result_t Meas_WeightAvgSizeCh0Q(scpi_t *context) { return Meas_WeightAvgSizeChQ(context, 0); }
static scpi_result_t Meas_WeightAvgSizeCh1Q(scpi_t *context) { return Meas_WeightAvgSizeChQ(context, 1); }

/* --- Standard deviation --- */

static scpi_result_t Meas_WeightSdevChQ(scpi_t *context, int ch) {
    taskENTER_CRITICAL(&measMux);
    float sd = extADCRunAV[ch].getStandardDeviation();
    taskEXIT_CRITICAL(&measMux);
    SCPI_ResultFloat(context, sd);
    return SCPI_RES_OK;
}
static scpi_result_t Meas_WeightSdevCh0Q(scpi_t *context) { return Meas_WeightSdevChQ(context, 0); }
static scpi_result_t Meas_WeightSdevCh1Q(scpi_t *context) { return Meas_WeightSdevChQ(context, 1); }

/* --- Stability --- */

static scpi_result_t Meas_WeightStableChQ(scpi_t *context, int ch) {
    taskENTER_CRITICAL(&measMux);
    float sd   = extADCRunAV[ch].getStandardDeviation();
    bool  full = extADCRunAV[ch].bufferIsFull();
    taskEXIT_CRITICAL(&measMux);
    SCPI_ResultBool(context, full && sd < (stabThreshold[ch] * overloadCapacity[ch]));
    return SCPI_RES_OK;
}
static scpi_result_t Meas_WeightStableCh0Q(scpi_t *context) { return Meas_WeightStableChQ(context, 0); }
static scpi_result_t Meas_WeightStableCh1Q(scpi_t *context) { return Meas_WeightStableChQ(context, 1); }

/* --- Gross weight (before tare) --- */

static scpi_result_t Meas_WeightGrossChQ(scpi_t *context, int ch) {
    taskENTER_CRITICAL(&measMux);
    float avg = extADCRunAV[ch].getAverage();
    taskEXIT_CRITICAL(&measMux);
    /* avg is already in display units (after tare subtraction).
       tareValue is in calUnit — convert to display units. */
    float tareDisplay = tareValue[ch] * unitConversionFactor(calUnit[ch], unitVal[ch]);
    float gross = avg + tareDisplay;
    SCPI_ResultFloat(context, gross);
    return SCPI_RES_OK;
}
static scpi_result_t Meas_WeightGrossCh0Q(scpi_t *context) { return Meas_WeightGrossChQ(context, 0); }
static scpi_result_t Meas_WeightGrossCh1Q(scpi_t *context) { return Meas_WeightGrossChQ(context, 1); }

/* --- Overload --- */

static scpi_result_t Meas_WeightOverloadChQ(scpi_t *context, int ch) {
    taskENTER_CRITICAL(&measMux);
    float avg = extADCRunAV[ch].getAverage();
    taskEXIT_CRITICAL(&measMux);
    SCPI_ResultBool(context, fabsf(avg) > overloadCapacity[ch]);
    return SCPI_RES_OK;
}
static scpi_result_t Meas_WeightOverloadCh0Q(scpi_t *context) { return Meas_WeightOverloadChQ(context, 0); }
static scpi_result_t Meas_WeightOverloadCh1Q(scpi_t *context) { return Meas_WeightOverloadChQ(context, 1); }

/* ------------------------------------------------------------------ */
/*  Configuration commands — per-channel                              */
/* ------------------------------------------------------------------ */

/* --- UNIT --- */

static scpi_result_t Conf_UnitCh(scpi_t *context, int ch) {
    int32_t val;
    if (!SCPI_ParamChoice(context, unit_choices, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    e_unitVal newUnit = (e_unitVal) val;
    taskENTER_CRITICAL(&measMux);
    if (newUnit != unitVal[ch]) {
        float convFactor = unitConversionFactor(unitVal[ch], newUnit);
        overloadCapacity[ch] *= convFactor;
        extADCweightMax[ch] *= convFactor;
        extADCRunAV[ch].clear();
        unitVal[ch] = newUnit;
    }
    taskEXIT_CRITICAL(&measMux);
    return SCPI_RES_OK;
}
static scpi_result_t Conf_UnitCh0(scpi_t *context) { return Conf_UnitCh(context, 0); }
static scpi_result_t Conf_UnitCh1(scpi_t *context) { return Conf_UnitCh(context, 1); }

static scpi_result_t Conf_UnitChQ(scpi_t *context, int ch) {
    const char *name = NULL;
    SCPI_ChoiceToName(unit_choices, (int32_t) unitVal[ch], &name);
    if (!name) name = "?";
    SCPI_ResultCharacters(context, name, strlen(name));
    return SCPI_RES_OK;
}
static scpi_result_t Conf_UnitCh0Q(scpi_t *context) { return Conf_UnitChQ(context, 0); }
static scpi_result_t Conf_UnitCh1Q(scpi_t *context) { return Conf_UnitChQ(context, 1); }

/* --- TARE --- */

static scpi_result_t Conf_TareCh(scpi_t *context, int ch) {
    scpi_number_t val;
    if (SCPI_ParamNumber(context, scpi_special_numbers_def, &val, FALSE)) {
        /* Explicit tare value provided */
        tareValue[ch] = (float) val.content.value;
    } else {
        /* No parameter — auto-tare from current reading */
        if (calValue[ch] == 0.0f) {
            SCPI_ErrorPush(context, SCPI_ERROR_EXECUTION_ERROR);
            return SCPI_RES_ERR;
        }
        tareValue[ch] = (extADCResultCh[ch] - zeroValue[ch]) / calValue[ch];
    }
    extADCRunAV[ch].clear();
    return SCPI_RES_OK;
}
static scpi_result_t Conf_TareCh0(scpi_t *context) { return Conf_TareCh(context, 0); }
static scpi_result_t Conf_TareCh1(scpi_t *context) { return Conf_TareCh(context, 1); }

static scpi_result_t Conf_TareChQ(scpi_t *context, int ch) {
    SCPI_ResultFloat(context, tareValue[ch]);
    return SCPI_RES_OK;
}
static scpi_result_t Conf_TareCh0Q(scpi_t *context) { return Conf_TareChQ(context, 0); }
static scpi_result_t Conf_TareCh1Q(scpi_t *context) { return Conf_TareChQ(context, 1); }

/* --- ZERO (quick zero from current ADC reading) --- */

static scpi_result_t Conf_ZeroCh(scpi_t *context, int ch) {
    (void) context;
    zeroValue[ch] = extADCResultCh[ch];
    tareValue[ch] = 0;
    extADCRunAV[ch].clear();
    EEPROM.put(eepromAddrZeroValue(ch), zeroValue[ch]);
    EEPROM.commit();
    return SCPI_RES_OK;
}
static scpi_result_t Conf_ZeroCh0(scpi_t *context) { return Conf_ZeroCh(context, 0); }
static scpi_result_t Conf_ZeroCh1(scpi_t *context) { return Conf_ZeroCh(context, 1); }

/* ------------------------------------------------------------------ */
/*  ADC configuration commands (shared — affect physical ADC)         */
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

/* CONFigure:ADC:INVert:CHx <ON|OFF> — per-channel ADC polarity inversion.
   Negates zeroValue and tareValue so calibration is preserved and
   weight_new = -weight_old.  No re-calibration required. */
static scpi_result_t Conf_AdcInvertCh(scpi_t *context, int ch) {
    scpi_bool_t val;
    if (!SCPI_ParamBool(context, &val, TRUE)) return SCPI_RES_ERR;
    bool newInvert = (bool) val;

    taskENTER_CRITICAL(&measMux);
    if (newInvert != adcInvert[ch]) {
        adcInvert[ch] = newInvert;
        zeroValue[ch] = -zeroValue[ch];
        tareValue[ch] = -tareValue[ch];
        extADCweightMax[ch] = -extADCweightMax[ch];
        extADCRunAV[ch].clear();
    }
    taskEXIT_CRITICAL(&measMux);

    EEPROM.put(eepromAddrAdcInvert(ch), (uint8_t) adcInvert[ch]);
    EEPROM.put(eepromAddrZeroValue(ch), zeroValue[ch]);
    EEPROM.put(eepromAddrWeightMax(ch), extADCweightMax[ch]);
    EEPROM.commit();
    return SCPI_RES_OK;
}
static scpi_result_t Conf_AdcInvertCh0(scpi_t *context) { return Conf_AdcInvertCh(context, 0); }
static scpi_result_t Conf_AdcInvertCh1(scpi_t *context) { return Conf_AdcInvertCh(context, 1); }

static scpi_result_t Conf_AdcInvertChQ(scpi_t *context, int ch) {
    SCPI_ResultBool(context, adcInvert[ch]);
    return SCPI_RES_OK;
}
static scpi_result_t Conf_AdcInvertCh0Q(scpi_t *context) { return Conf_AdcInvertChQ(context, 0); }
static scpi_result_t Conf_AdcInvertCh1Q(scpi_t *context) { return Conf_AdcInvertChQ(context, 1); }

/* CONFigure:STABility:THReshold:CHx <val> — per-channel stability threshold (persists) */
static scpi_result_t Conf_StabThreshCh(scpi_t *context, int ch) {
    scpi_number_t val;
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    if (val.content.value <= 0.0 || val.content.value > 1.0) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    stabThreshold[ch] = (float) val.content.value;
    EEPROM.put(eepromAddrStabThresh(ch), stabThreshold[ch]);
    EEPROM.commit();
    return SCPI_RES_OK;
}
static scpi_result_t Conf_StabThreshCh0(scpi_t *context) { return Conf_StabThreshCh(context, 0); }
static scpi_result_t Conf_StabThreshCh1(scpi_t *context) { return Conf_StabThreshCh(context, 1); }

static scpi_result_t Conf_StabThreshChQ(scpi_t *context, int ch) {
    SCPI_ResultFloat(context, stabThreshold[ch]);
    return SCPI_RES_OK;
}
static scpi_result_t Conf_StabThreshCh0Q(scpi_t *context) { return Conf_StabThreshChQ(context, 0); }
static scpi_result_t Conf_StabThreshCh1Q(scpi_t *context) { return Conf_StabThreshChQ(context, 1); }

/* --- Overload capacity — per-channel --- */

static scpi_result_t Conf_OverCapCh(scpi_t *context, int ch) {
    scpi_number_t val;
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    if (val.content.value <= 0.0 || val.content.value > (double)FLT_MAX) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    overloadCapacity[ch] = (float) val.content.value;
    EEPROM.put(eepromAddrOverCap(ch), overloadCapacity[ch]);
    EEPROM.put(eepromAddrUnitVal(ch), unitVal[ch]);  // keep unit+capacity in sync
    EEPROM.commit();
    return SCPI_RES_OK;
}
static scpi_result_t Conf_OverCapCh0(scpi_t *context) { return Conf_OverCapCh(context, 0); }
static scpi_result_t Conf_OverCapCh1(scpi_t *context) { return Conf_OverCapCh(context, 1); }

static scpi_result_t Conf_OverCapChQ(scpi_t *context, int ch) {
    SCPI_ResultFloat(context, overloadCapacity[ch]);
    return SCPI_RES_OK;
}
static scpi_result_t Conf_OverCapCh0Q(scpi_t *context) { return Conf_OverCapChQ(context, 0); }
static scpi_result_t Conf_OverCapCh1Q(scpi_t *context) { return Conf_OverCapChQ(context, 1); }

/* --- Adaptive filter (per-channel) --- */

/* CONFigure:FILTer:ADAPtive:CHx <ON|OFF> */
static scpi_result_t Conf_AdaptEnableCh(scpi_t *context, int ch) {
    scpi_bool_t val;
    if (!SCPI_ParamBool(context, &val, TRUE)) return SCPI_RES_ERR;
    adaptiveFilterEnable[ch] = (bool) val;
    EEPROM.put(eepromAddrAdaptEnable(ch), (uint8_t) adaptiveFilterEnable[ch]);
    EEPROM.commit();
    return SCPI_RES_OK;
}
static scpi_result_t Conf_AdaptEnableCh0(scpi_t *context) { return Conf_AdaptEnableCh(context, 0); }
static scpi_result_t Conf_AdaptEnableCh1(scpi_t *context) { return Conf_AdaptEnableCh(context, 1); }

static scpi_result_t Conf_AdaptEnableChQ(scpi_t *context, int ch) {
    SCPI_ResultBool(context, adaptiveFilterEnable[ch]);
    return SCPI_RES_OK;
}
static scpi_result_t Conf_AdaptEnableCh0Q(scpi_t *context) { return Conf_AdaptEnableChQ(context, 0); }
static scpi_result_t Conf_AdaptEnableCh1Q(scpi_t *context) { return Conf_AdaptEnableChQ(context, 1); }

/* CONFigure:FILTer:ADAPtive:THReshold:CHx <val> — threshold as % of capacity */
static scpi_result_t Conf_AdaptThreshCh(scpi_t *context, int ch) {
    scpi_number_t val;
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    if (val.content.value <= 0.0 || val.content.value > 100.0) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    adaptiveFilterPct[ch] = (float) val.content.value;
    EEPROM.put(eepromAddrAdaptThresh(ch), adaptiveFilterPct[ch]);
    EEPROM.commit();
    return SCPI_RES_OK;
}
static scpi_result_t Conf_AdaptThreshCh0(scpi_t *context) { return Conf_AdaptThreshCh(context, 0); }
static scpi_result_t Conf_AdaptThreshCh1(scpi_t *context) { return Conf_AdaptThreshCh(context, 1); }

static scpi_result_t Conf_AdaptThreshChQ(scpi_t *context, int ch) {
    SCPI_ResultFloat(context, adaptiveFilterPct[ch]);
    return SCPI_RES_OK;
}
static scpi_result_t Conf_AdaptThreshCh0Q(scpi_t *context) { return Conf_AdaptThreshChQ(context, 0); }
static scpi_result_t Conf_AdaptThreshCh1Q(scpi_t *context) { return Conf_AdaptThreshChQ(context, 1); }

/* CONFigure:FILTer:ADAPtive:TIME:CHx <val> — sustained deviation window in us */
static scpi_result_t Conf_AdaptTimeCh(scpi_t *context, int ch) {
    int32_t val;
    if (!SCPI_ParamInt32(context, &val, TRUE)) return SCPI_RES_ERR;
    if (val < 1 || val > 60000000) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    adaptiveFilterTimeUs[ch] = (uint32_t) val;
    EEPROM.put(eepromAddrAdaptTime(ch), adaptiveFilterTimeUs[ch]);
    EEPROM.commit();
    return SCPI_RES_OK;
}
static scpi_result_t Conf_AdaptTimeCh0(scpi_t *context) { return Conf_AdaptTimeCh(context, 0); }
static scpi_result_t Conf_AdaptTimeCh1(scpi_t *context) { return Conf_AdaptTimeCh(context, 1); }

static scpi_result_t Conf_AdaptTimeChQ(scpi_t *context, int ch) {
    SCPI_ResultInt32(context, (int32_t) adaptiveFilterTimeUs[ch]);
    return SCPI_RES_OK;
}
static scpi_result_t Conf_AdaptTimeCh0Q(scpi_t *context) { return Conf_AdaptTimeChQ(context, 0); }
static scpi_result_t Conf_AdaptTimeCh1Q(scpi_t *context) { return Conf_AdaptTimeChQ(context, 1); }

/* ------------------------------------------------------------------ */
/*  Calibration commands — per-channel                                */
/* ------------------------------------------------------------------ */

/* --- CAL:VALUE --- */

static scpi_result_t Cal_ValueChQ(scpi_t *context, int ch) {
    SCPI_ResultFloat(context, calValue[ch]);
    return SCPI_RES_OK;
}
static scpi_result_t Cal_ValueCh0Q(scpi_t *context) { return Cal_ValueChQ(context, 0); }
static scpi_result_t Cal_ValueCh1Q(scpi_t *context) { return Cal_ValueChQ(context, 1); }

static scpi_result_t Cal_ValueCh(scpi_t *context, int ch) {
    scpi_number_t val;
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    if (val.content.value <= 0.0 || val.content.value > (double)FLT_MAX) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    calValue[ch] = (float) val.content.value;
    EEPROM.put(eepromAddrCalValue(ch), calValue[ch]);
    EEPROM.commit();
    return SCPI_RES_OK;
}
static scpi_result_t Cal_ValueCh0(scpi_t *context) { return Cal_ValueCh(context, 0); }
static scpi_result_t Cal_ValueCh1(scpi_t *context) { return Cal_ValueCh(context, 1); }

/* --- CAL:ZERO --- */

static scpi_result_t Cal_ZeroChQ(scpi_t *context, int ch) {
    SCPI_ResultInt32(context, zeroValue[ch]);
    return SCPI_RES_OK;
}
static scpi_result_t Cal_ZeroCh0Q(scpi_t *context) { return Cal_ZeroChQ(context, 0); }
static scpi_result_t Cal_ZeroCh1Q(scpi_t *context) { return Cal_ZeroChQ(context, 1); }

static scpi_result_t Cal_ZeroCh(scpi_t *context, int ch) {
    int32_t val;
    if (!SCPI_ParamInt32(context, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    zeroValue[ch] = val;
    EEPROM.put(eepromAddrZeroValue(ch), zeroValue[ch]);
    EEPROM.commit();
    return SCPI_RES_OK;
}
static scpi_result_t Cal_ZeroCh0(scpi_t *context) { return Cal_ZeroCh(context, 0); }
static scpi_result_t Cal_ZeroCh1(scpi_t *context) { return Cal_ZeroCh(context, 1); }

/* --- CAL:WEIGHT --- */

static scpi_result_t Cal_WeightChQ(scpi_t *context, int ch) {
    SCPI_ResultUInt32(context, calWeight[ch]);
    return SCPI_RES_OK;
}
static scpi_result_t Cal_WeightCh0Q(scpi_t *context) { return Cal_WeightChQ(context, 0); }
static scpi_result_t Cal_WeightCh1Q(scpi_t *context) { return Cal_WeightChQ(context, 1); }

static scpi_result_t Cal_WeightCh(scpi_t *context, int ch) {
    int32_t val;
    if (!SCPI_ParamInt32(context, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    if (val <= 0) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    calWeight[ch] = (uint32_t) val;
    EEPROM.put(eepromAddrCalWeight(ch), calWeight[ch]);
    EEPROM.commit();
    return SCPI_RES_OK;
}
static scpi_result_t Cal_WeightCh0(scpi_t *context) { return Cal_WeightCh(context, 0); }
static scpi_result_t Cal_WeightCh1(scpi_t *context) { return Cal_WeightCh(context, 1); }

/* --- CAL:UNIT --- */

static scpi_result_t Cal_UnitChQ(scpi_t *context, int ch) {
    const char *name = NULL;
    SCPI_ChoiceToName(unit_choices, (int32_t) calUnit[ch], &name);
    if (!name) name = "?";
    SCPI_ResultCharacters(context, name, strlen(name));
    return SCPI_RES_OK;
}
static scpi_result_t Cal_UnitCh0Q(scpi_t *context) { return Cal_UnitChQ(context, 0); }
static scpi_result_t Cal_UnitCh1Q(scpi_t *context) { return Cal_UnitChQ(context, 1); }

static scpi_result_t Cal_UnitCh(scpi_t *context, int ch) {
    int32_t val;
    if (!SCPI_ParamChoice(context, unit_choices, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    calUnit[ch] = (e_unitVal) val;
    EEPROM.put(eepromAddrCalUnit(ch), calUnit[ch]);
    EEPROM.commit();
    return SCPI_RES_OK;
}
static scpi_result_t Cal_UnitCh0(scpi_t *context) { return Cal_UnitCh(context, 0); }
static scpi_result_t Cal_UnitCh1(scpi_t *context) { return Cal_UnitCh(context, 1); }

/* ------------------------------------------------------------------ */
/*  Remote calibration commands (blocking) — per-channel              */
/* ------------------------------------------------------------------ */

/* Helper: clear running average for channel and block until buffer is
   full (settled).  Returns true on success, false on timeout (30 s). */
static bool waitForSettle(int ch) {
    taskENTER_CRITICAL(&measMux);
    extADCRunAV[ch].clear();
    taskEXIT_CRITICAL(&measMux);

    const TickType_t timeout = pdMS_TO_TICKS(30000);
    TickType_t start = xTaskGetTickCount();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        taskENTER_CRITICAL(&measMux);
        bool full = extADCRunAV[ch].bufferIsFull();
        taskEXIT_CRITICAL(&measMux);
        if (full) return true;
        if ((xTaskGetTickCount() - start) >= timeout) return false;
    }
}

/* CAL:ZERO:EXEC — block until settled, capture zero, persist */
static scpi_result_t Cal_ZeroExecCh(scpi_t *context, int ch) {
    if (!waitForSettle(ch)) {
        SCPI_ErrorPush(context, SCPI_ERROR_EXECUTION_ERROR);
        return SCPI_RES_ERR;
    }
    zeroValue[ch] = extADCResultCh[ch];
    tareValue[ch] = 0;
    EEPROM.put(eepromAddrZeroValue(ch), zeroValue[ch]);
    EEPROM.commit();
    taskENTER_CRITICAL(&measMux);
    extADCRunAV[ch].clear();
    taskEXIT_CRITICAL(&measMux);
    DBG_PRINTF("CAL:ZERO:EXEC CH%d zeroValue=%ld\r\n", ch, (long) zeroValue[ch]);
    return SCPI_RES_OK;
}
static scpi_result_t Cal_ZeroExecCh0(scpi_t *context) { return Cal_ZeroExecCh(context, 0); }
static scpi_result_t Cal_ZeroExecCh1(scpi_t *context) { return Cal_ZeroExecCh(context, 1); }

/* CAL:SPAN:EXEC — block until settled, compute calValue, persist */
static scpi_result_t Cal_SpanExecCh(scpi_t *context, int ch) {
    if (calWeight[ch] == 0) {
        SCPI_ErrorPush(context, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    if (!waitForSettle(ch)) {
        SCPI_ErrorPush(context, SCPI_ERROR_EXECUTION_ERROR);
        return SCPI_RES_ERR;
    }
    float newCalValue = (float)(extADCResultCh[ch] - zeroValue[ch]) / (float) calWeight[ch];
    if (newCalValue <= 0.0f) {
        SCPI_ErrorPush(context, SCPI_ERROR_EXECUTION_ERROR);
        return SCPI_RES_ERR;
    }
    calValue[ch] = newCalValue;
    EEPROM.put(eepromAddrCalValue(ch), calValue[ch]);
    EEPROM.put(eepromAddrZeroValue(ch), zeroValue[ch]);
    EEPROM.put(eepromAddrCalWeight(ch), calWeight[ch]);
    EEPROM.put(eepromAddrCalUnit(ch), calUnit[ch]);
    EEPROM.commit();
    taskENTER_CRITICAL(&measMux);
    extADCRunAV[ch].clear();
    taskEXIT_CRITICAL(&measMux);
    DBG_PRINTF("CAL:SPAN:EXEC CH%d calValue=%.6f\r\n", ch, (double) calValue[ch]);
    return SCPI_RES_OK;
}
static scpi_result_t Cal_SpanExecCh0(scpi_t *context) { return Cal_SpanExecCh(context, 0); }
static scpi_result_t Cal_SpanExecCh1(scpi_t *context) { return Cal_SpanExecCh(context, 1); }

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
/* CONFigure:DISPlay:MODE <SINGle|DUAL|SUM> — set display mode (persists) */
static scpi_result_t Conf_DispMode(scpi_t *context) {
    int32_t val;
    if (!SCPI_ParamChoice(context, display_mode_choices, &val, TRUE)) {
        return SCPI_RES_ERR;
    }
    displayMode = (e_displayMode) val;
    EEPROM.put(EEPROM_ADDR_DISP_MODE, (uint8_t) displayMode);
    EEPROM.commit();
    return SCPI_RES_OK;
}

/* CONFigure:DISPlay:MODE? */
static scpi_result_t Conf_DispModeQ(scpi_t *context) {
    const char *name = NULL;
    SCPI_ChoiceToName(display_mode_choices, (int32_t) displayMode, &name);
    if (!name) name = "?";
    SCPI_ResultCharacters(context, name, strlen(name));
    return SCPI_RES_OK;
}

extern void powerDown(void);
extern void wakeFromIdleMode(void);
static scpi_result_t Sys_PowerDown(scpi_t *context) {
    (void) context;
    powerDown();
    return SCPI_RES_OK;  /* may not be reached */
}

/* SYSTem:EEPROM? — dump all persisted EEPROM values (both channels) */
static scpi_result_t Sys_EepromQ(scpi_t *context) {
    /* --- Shared values --- */
    e_backlightEnable ee_backlight; EEPROM.get(EEPROM_ADDR_BACKLIGHT,  ee_backlight);
    uint8_t          ee_backlightPWM; EEPROM.get(EEPROM_ADDR_BACKLIGHT_PWM, ee_backlightPWM);
    uint8_t          ee_echo;   EEPROM.get(EEPROM_ADDR_ECHO,   ee_echo);
    uint8_t          ee_prompt; EEPROM.get(EEPROM_ADDR_PROMPT,  ee_prompt);

    /* --- Per-channel values --- */
    float    ee_calValue[NUM_CHANNELS];
    int32_t  ee_zeroValue[NUM_CHANNELS];
    e_unitVal ee_unit[NUM_CHANNELS];
    uint32_t ee_calWeight[NUM_CHANNELS];
    e_unitVal ee_calUnit[NUM_CHANNELS];
    float    ee_weightMax[NUM_CHANNELS];
    float    ee_overCap[NUM_CHANNELS];
    float    ee_stabThresh[NUM_CHANNELS];
    uint8_t  ee_adaptEn[NUM_CHANNELS];
    float    ee_adaptThr[NUM_CHANNELS];
    uint32_t ee_adaptTime[NUM_CHANNELS];
    uint8_t  ee_adcInvert[NUM_CHANNELS];
    for (int c = 0; c < NUM_CHANNELS; c++) {
        EEPROM.get(eepromAddrCalValue(c),  ee_calValue[c]);
        EEPROM.get(eepromAddrZeroValue(c), ee_zeroValue[c]);
        EEPROM.get(eepromAddrUnitVal(c),   ee_unit[c]);
        EEPROM.get(eepromAddrCalWeight(c), ee_calWeight[c]);
        EEPROM.get(eepromAddrCalUnit(c),   ee_calUnit[c]);
        EEPROM.get(eepromAddrWeightMax(c), ee_weightMax[c]);
        EEPROM.get(eepromAddrOverCap(c),   ee_overCap[c]);
        EEPROM.get(eepromAddrStabThresh(c), ee_stabThresh[c]);
        EEPROM.get(eepromAddrAdaptEnable(c), ee_adaptEn[c]);
        EEPROM.get(eepromAddrAdaptThresh(c), ee_adaptThr[c]);
        EEPROM.get(eepromAddrAdaptTime(c),   ee_adaptTime[c]);
        EEPROM.get(eepromAddrAdcInvert(c),   ee_adcInvert[c]);
    }

    const char *unit_name[NUM_CHANNELS], *cal_unit_name[NUM_CHANNELS];
    for (int c = 0; c < NUM_CHANNELS; c++) {
        unit_name[c] = NULL;
        cal_unit_name[c] = NULL;
        SCPI_ChoiceToName(unit_choices, (int32_t) ee_unit[c], &unit_name[c]);
        SCPI_ChoiceToName(unit_choices, (int32_t) ee_calUnit[c], &cal_unit_name[c]);
    }

    char buf[1024];
    snprintf(buf, sizeof(buf),
        "CH0:{calValue=%.6f,zeroValue=%ld,unit=%s,"
        "calWeight=%lu,calUnit=%s,weightMax=%.4f,overCap=%.4f,"
        "stabThresh=%.4f,adaptEn=%u,adaptThr=%.4f,adaptTimeUs=%lu,adcInvert=%u},"
        "CH1:{calValue=%.6f,zeroValue=%ld,unit=%s,"
        "calWeight=%lu,calUnit=%s,weightMax=%.4f,overCap=%.4f,"
        "stabThresh=%.4f,adaptEn=%u,adaptThr=%.4f,adaptTimeUs=%lu,adcInvert=%u},"
        "backlight=%d,backlightPWM=%u,echo=%u,prompt=%u",
        /* CH0 */
        (double) ee_calValue[0], (long) ee_zeroValue[0],
        unit_name[0] ? unit_name[0] : "?",
        (unsigned long) ee_calWeight[0], cal_unit_name[0] ? cal_unit_name[0] : "?",
        (double) ee_weightMax[0], (double) ee_overCap[0],
        (double) ee_stabThresh[0], (unsigned) ee_adaptEn[0],
        (double) ee_adaptThr[0], (unsigned long) ee_adaptTime[0],
        (unsigned) ee_adcInvert[0],
        /* CH1 */
        (double) ee_calValue[1], (long) ee_zeroValue[1],
        unit_name[1] ? unit_name[1] : "?",
        (unsigned long) ee_calWeight[1], cal_unit_name[1] ? cal_unit_name[1] : "?",
        (double) ee_weightMax[1], (double) ee_overCap[1],
        (double) ee_stabThresh[1], (unsigned) ee_adaptEn[1],
        (double) ee_adaptThr[1], (unsigned long) ee_adaptTime[1],
        (unsigned) ee_adcInvert[1],
        /* Shared */
        (int) ee_backlight, (unsigned) ee_backlightPWM,
        (unsigned) ee_echo, (unsigned) ee_prompt);
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

    /* ---- Measurement: per-channel ---- */

    /* Weight (averaged, unit-converted) */
    { .pattern = "MEASure:WEIGht:CH0?",                  .callback = Meas_WeightCh0Q, },
    { .pattern = "MEASure:WEIGht:CH1?",                  .callback = Meas_WeightCh1Q, },
    { .pattern = "MEASure:WEIGht:SUM?",                  .callback = Meas_WeightSumQ, },

    /* Raw ADC counts */
    { .pattern = "MEASure:WEIGht:RAW:CH0?",              .callback = Meas_WeightRawCh0Q, },
    { .pattern = "MEASure:WEIGht:RAW:CH1?",              .callback = Meas_WeightRawCh1Q, },
    { .pattern = "MEASure:WEIGht:RAW:SUM?",              .callback = Meas_WeightRawSumQ, },

    /* Peak / Max */
    { .pattern = "MEASure:WEIGht:MAX:CH0",               .callback = Meas_WeightMaxCh0, },
    { .pattern = "MEASure:WEIGht:MAX:CH0?",              .callback = Meas_WeightMaxCh0Q, },
    { .pattern = "MEASure:WEIGht:MAX:CH1",               .callback = Meas_WeightMaxCh1, },
    { .pattern = "MEASure:WEIGht:MAX:CH1?",              .callback = Meas_WeightMaxCh1Q, },

    /* Average count */
    { .pattern = "MEASure:WEIGht:AVERage:COUNt:CH0?",    .callback = Meas_WeightAvgCountCh0Q, },
    { .pattern = "MEASure:WEIGht:AVERage:COUNt:CH1?",    .callback = Meas_WeightAvgCountCh1Q, },

    /* Average size */
    { .pattern = "MEASure:WEIGht:AVERage:SIZE:CH0?",     .callback = Meas_WeightAvgSizeCh0Q, },
    { .pattern = "MEASure:WEIGht:AVERage:SIZE:CH1?",     .callback = Meas_WeightAvgSizeCh1Q, },

    /* Standard deviation */
    { .pattern = "MEASure:WEIGht:SDEViation:CH0?",       .callback = Meas_WeightSdevCh0Q, },
    { .pattern = "MEASure:WEIGht:SDEViation:CH1?",       .callback = Meas_WeightSdevCh1Q, },

    /* Stability */
    { .pattern = "MEASure:WEIGht:STABle:CH0?",           .callback = Meas_WeightStableCh0Q, },
    { .pattern = "MEASure:WEIGht:STABle:CH1?",           .callback = Meas_WeightStableCh1Q, },

    /* Gross weight */
    { .pattern = "MEASure:WEIGht:GROSs:CH0?",            .callback = Meas_WeightGrossCh0Q, },
    { .pattern = "MEASure:WEIGht:GROSs:CH1?",            .callback = Meas_WeightGrossCh1Q, },

    /* Overload */
    { .pattern = "MEASure:WEIGht:OVERload:CH0?",         .callback = Meas_WeightOverloadCh0Q, },
    { .pattern = "MEASure:WEIGht:OVERload:CH1?",         .callback = Meas_WeightOverloadCh1Q, },

    /* ---- Force aliases (same callbacks as WEIGht) ---- */
    { .pattern = "MEASure:FORCe:CH0?",                    .callback = Meas_WeightCh0Q, },
    { .pattern = "MEASure:FORCe:CH1?",                    .callback = Meas_WeightCh1Q, },
    { .pattern = "MEASure:FORCe:SUM?",                    .callback = Meas_WeightSumQ, },
    { .pattern = "MEASure:FORCe:RAW:CH0?",                .callback = Meas_WeightRawCh0Q, },
    { .pattern = "MEASure:FORCe:RAW:CH1?",                .callback = Meas_WeightRawCh1Q, },
    { .pattern = "MEASure:FORCe:RAW:SUM?",                .callback = Meas_WeightRawSumQ, },
    { .pattern = "MEASure:FORCe:MAX:CH0",                 .callback = Meas_WeightMaxCh0, },
    { .pattern = "MEASure:FORCe:MAX:CH0?",                .callback = Meas_WeightMaxCh0Q, },
    { .pattern = "MEASure:FORCe:MAX:CH1",                 .callback = Meas_WeightMaxCh1, },
    { .pattern = "MEASure:FORCe:MAX:CH1?",                .callback = Meas_WeightMaxCh1Q, },
    { .pattern = "MEASure:FORCe:AVERage:COUNt:CH0?",      .callback = Meas_WeightAvgCountCh0Q, },
    { .pattern = "MEASure:FORCe:AVERage:COUNt:CH1?",      .callback = Meas_WeightAvgCountCh1Q, },
    { .pattern = "MEASure:FORCe:AVERage:SIZE:CH0?",       .callback = Meas_WeightAvgSizeCh0Q, },
    { .pattern = "MEASure:FORCe:AVERage:SIZE:CH1?",       .callback = Meas_WeightAvgSizeCh1Q, },
    { .pattern = "MEASure:FORCe:SDEViation:CH0?",         .callback = Meas_WeightSdevCh0Q, },
    { .pattern = "MEASure:FORCe:SDEViation:CH1?",         .callback = Meas_WeightSdevCh1Q, },
    { .pattern = "MEASure:FORCe:STABle:CH0?",             .callback = Meas_WeightStableCh0Q, },
    { .pattern = "MEASure:FORCe:STABle:CH1?",             .callback = Meas_WeightStableCh1Q, },
    { .pattern = "MEASure:FORCe:GROSs:CH0?",              .callback = Meas_WeightGrossCh0Q, },
    { .pattern = "MEASure:FORCe:GROSs:CH1?",              .callback = Meas_WeightGrossCh1Q, },
    { .pattern = "MEASure:FORCe:OVERload:CH0?",           .callback = Meas_WeightOverloadCh0Q, },
    { .pattern = "MEASure:FORCe:OVERload:CH1?",           .callback = Meas_WeightOverloadCh1Q, },

    /* ---- Configuration: per-channel ---- */

    { .pattern = "CONFigure:UNIT:CH0",                    .callback = Conf_UnitCh0, },
    { .pattern = "CONFigure:UNIT:CH0?",                   .callback = Conf_UnitCh0Q, },
    { .pattern = "CONFigure:UNIT:CH1",                    .callback = Conf_UnitCh1, },
    { .pattern = "CONFigure:UNIT:CH1?",                   .callback = Conf_UnitCh1Q, },

    { .pattern = "CONFigure:TARE:CH0",                    .callback = Conf_TareCh0, },
    { .pattern = "CONFigure:TARE:CH0?",                   .callback = Conf_TareCh0Q, },
    { .pattern = "CONFigure:TARE:CH1",                    .callback = Conf_TareCh1, },
    { .pattern = "CONFigure:TARE:CH1?",                   .callback = Conf_TareCh1Q, },

    { .pattern = "CONFigure:ZERO:CH0",                    .callback = Conf_ZeroCh0, },
    { .pattern = "CONFigure:ZERO:CH1",                    .callback = Conf_ZeroCh1, },

    /* Display mode */
    { .pattern = "CONFigure:DISPlay:MODE",                .callback = Conf_DispMode, },
    { .pattern = "CONFigure:DISPlay:MODE?",               .callback = Conf_DispModeQ, },

    /* ADC configuration (shared — physical ADC) */
    { .pattern = "CONFigure:ADC:RATE",                    .callback = Conf_AdcRate, },
    { .pattern = "CONFigure:ADC:RATE?",                   .callback = Conf_AdcRateQ, },
    { .pattern = "CONFigure:ADC:FILTer",                  .callback = Conf_AdcFilter, },
    { .pattern = "CONFigure:ADC:FILTer?",                 .callback = Conf_AdcFilterQ, },
    { .pattern = "CONFigure:ADC:NOTCh",                   .callback = Conf_AdcNotch, },
    { .pattern = "CONFigure:ADC:NOTCh?",                  .callback = Conf_AdcNotchQ, },
    /* ADC invert (per-channel) */
    { .pattern = "CONFigure:ADC:INVert:CH0",              .callback = Conf_AdcInvertCh0, },
    { .pattern = "CONFigure:ADC:INVert:CH0?",             .callback = Conf_AdcInvertCh0Q, },
    { .pattern = "CONFigure:ADC:INVert:CH1",              .callback = Conf_AdcInvertCh1, },
    { .pattern = "CONFigure:ADC:INVert:CH1?",             .callback = Conf_AdcInvertCh1Q, },

    /* Stability threshold (per-channel) */
    { .pattern = "CONFigure:STABility:THReshold:CH0",     .callback = Conf_StabThreshCh0, },
    { .pattern = "CONFigure:STABility:THReshold:CH0?",    .callback = Conf_StabThreshCh0Q, },
    { .pattern = "CONFigure:STABility:THReshold:CH1",     .callback = Conf_StabThreshCh1, },
    { .pattern = "CONFigure:STABility:THReshold:CH1?",    .callback = Conf_StabThreshCh1Q, },

    /* Overload capacity (per-channel) */
    { .pattern = "CONFigure:OVERload:CAPacity:CH0",      .callback = Conf_OverCapCh0, },
    { .pattern = "CONFigure:OVERload:CAPacity:CH0?",     .callback = Conf_OverCapCh0Q, },
    { .pattern = "CONFigure:OVERload:CAPacity:CH1",      .callback = Conf_OverCapCh1, },
    { .pattern = "CONFigure:OVERload:CAPacity:CH1?",     .callback = Conf_OverCapCh1Q, },

    /* Adaptive filter (per-channel) */
    { .pattern = "CONFigure:FILTer:ADAPtive:CH0",             .callback = Conf_AdaptEnableCh0, },
    { .pattern = "CONFigure:FILTer:ADAPtive:CH0?",            .callback = Conf_AdaptEnableCh0Q, },
    { .pattern = "CONFigure:FILTer:ADAPtive:CH1",             .callback = Conf_AdaptEnableCh1, },
    { .pattern = "CONFigure:FILTer:ADAPtive:CH1?",            .callback = Conf_AdaptEnableCh1Q, },
    { .pattern = "CONFigure:FILTer:ADAPtive:THReshold:CH0",   .callback = Conf_AdaptThreshCh0, },
    { .pattern = "CONFigure:FILTer:ADAPtive:THReshold:CH0?",  .callback = Conf_AdaptThreshCh0Q, },
    { .pattern = "CONFigure:FILTer:ADAPtive:THReshold:CH1",   .callback = Conf_AdaptThreshCh1, },
    { .pattern = "CONFigure:FILTer:ADAPtive:THReshold:CH1?",  .callback = Conf_AdaptThreshCh1Q, },
    { .pattern = "CONFigure:FILTer:ADAPtive:TIME:CH0",        .callback = Conf_AdaptTimeCh0, },
    { .pattern = "CONFigure:FILTer:ADAPtive:TIME:CH0?",       .callback = Conf_AdaptTimeCh0Q, },
    { .pattern = "CONFigure:FILTer:ADAPtive:TIME:CH1",        .callback = Conf_AdaptTimeCh1, },
    { .pattern = "CONFigure:FILTer:ADAPtive:TIME:CH1?",       .callback = Conf_AdaptTimeCh1Q, },

    /* ---- Calibration: per-channel ---- */

    { .pattern = "CALibration:VALue:CH0",                 .callback = Cal_ValueCh0, },
    { .pattern = "CALibration:VALue:CH0?",                .callback = Cal_ValueCh0Q, },
    { .pattern = "CALibration:VALue:CH1",                 .callback = Cal_ValueCh1, },
    { .pattern = "CALibration:VALue:CH1?",                .callback = Cal_ValueCh1Q, },

    { .pattern = "CALibration:ZERO:CH0",                  .callback = Cal_ZeroCh0, },
    { .pattern = "CALibration:ZERO:CH0?",                 .callback = Cal_ZeroCh0Q, },
    { .pattern = "CALibration:ZERO:CH1",                  .callback = Cal_ZeroCh1, },
    { .pattern = "CALibration:ZERO:CH1?",                 .callback = Cal_ZeroCh1Q, },

    { .pattern = "CALibration:WEIGht:CH0",                .callback = Cal_WeightCh0, },
    { .pattern = "CALibration:WEIGht:CH0?",               .callback = Cal_WeightCh0Q, },
    { .pattern = "CALibration:WEIGht:CH1",                .callback = Cal_WeightCh1, },
    { .pattern = "CALibration:WEIGht:CH1?",               .callback = Cal_WeightCh1Q, },

    { .pattern = "CALibration:UNIT:CH0",                  .callback = Cal_UnitCh0, },
    { .pattern = "CALibration:UNIT:CH0?",                 .callback = Cal_UnitCh0Q, },
    { .pattern = "CALibration:UNIT:CH1",                  .callback = Cal_UnitCh1, },
    { .pattern = "CALibration:UNIT:CH1?",                 .callback = Cal_UnitCh1Q, },

    { .pattern = "CALibration:ZERO:EXEC:CH0",             .callback = Cal_ZeroExecCh0, },
    { .pattern = "CALibration:ZERO:EXEC:CH1",             .callback = Cal_ZeroExecCh1, },

    { .pattern = "CALibration:SPAN:EXEC:CH0",             .callback = Cal_SpanExecCh0, },
    { .pattern = "CALibration:SPAN:EXEC:CH1",             .callback = Cal_SpanExecCh1, },

    /* Force aliases for calibration weight */
    { .pattern = "CALibration:FORCe:CH0",                .callback = Cal_WeightCh0, },
    { .pattern = "CALibration:FORCe:CH0?",               .callback = Cal_WeightCh0Q, },
    { .pattern = "CALibration:FORCe:CH1",                .callback = Cal_WeightCh1, },
    { .pattern = "CALibration:FORCe:CH1?",               .callback = Cal_WeightCh1Q, },

    /* ---- System ---- */

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
