/*
 * scpi_interface.cpp — SCPI command handlers, interface callbacks, and FreeRTOS task
 */

#include <Arduino.h>
#include "appconfig.h"
#include "scpi_interface.h"
#include "RunningAverage.h"

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
extern const float kgtolbScalar;
extern const String unitAbbr[];
extern const uint8_t backlightPWM;

/* ------------------------------------------------------------------ */
/*  SCPI interface callbacks                                          */
/* ------------------------------------------------------------------ */

static size_t SCPI_Write(scpi_t *context, const char *data, size_t len) {
    (void) context;
    return Serial.write(reinterpret_cast<const uint8_t *>(data), len);
}

static scpi_result_t SCPI_Flush(scpi_t *context) {
    (void) context;
    Serial.flush();
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

/* ------------------------------------------------------------------ */
/*  Measurement commands                                              */
/* ------------------------------------------------------------------ */

/* MEASure:WEIGht? — return averaged, unit-converted weight */
static scpi_result_t Meas_WeightQ(scpi_t *context) {
    SCPI_ResultFloat(context, extADCRunAV.getAverage());
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
    SCPI_ResultFloat(context, extADCweightMax);
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
    unitVal = (e_unitVal) val;
    return SCPI_RES_OK;
}

/* CONFigure:UNIT? */
static scpi_result_t Conf_UnitQ(scpi_t *context) {
    const char *name;
    SCPI_ChoiceToName(unit_choices, (int32_t) unitVal, &name);
    SCPI_ResultCharacters(context, name, strlen(name));
    return SCPI_RES_OK;
}

/* CONFigure:TARE — perform tare */
static scpi_result_t Conf_Tare(scpi_t *context) {
    (void) context;
    tareValue = (extADCResult - zeroValue) / calValue;
    extADCRunAV.clear();
    return SCPI_RES_OK;
}

/* CONFigure:TARE? — return current tare value */
static scpi_result_t Conf_TareQ(scpi_t *context) {
    SCPI_ResultFloat(context, tareValue);
    return SCPI_RES_OK;
}

/* CONFigure:ZERO — set current reading as zero */
static scpi_result_t Conf_Zero(scpi_t *context) {
    (void) context;
    tareValue = (extADCResult - zeroValue) / calValue;
    extADCRunAV.clear();
    return SCPI_RES_OK;
}

/* ------------------------------------------------------------------ */
/*  Calibration read-only commands                                    */
/* ------------------------------------------------------------------ */

static scpi_result_t Cal_ValueQ(scpi_t *context) {
    SCPI_ResultFloat(context, calValue);
    return SCPI_RES_OK;
}

static scpi_result_t Cal_ZeroQ(scpi_t *context) {
    SCPI_ResultInt32(context, zeroValue);
    return SCPI_RES_OK;
}

static scpi_result_t Cal_WeightQ(scpi_t *context) {
    SCPI_ResultUInt32(context, calWeight);
    return SCPI_RES_OK;
}

static scpi_result_t Cal_UnitQ(scpi_t *context) {
    const char *name;
    SCPI_ChoiceToName(unit_choices, (int32_t) calUnit, &name);
    SCPI_ResultCharacters(context, name, strlen(name));
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
    ledcWrite(LCD_BACKLIGHT, backlightPWM * backlightEnable);
    return SCPI_RES_OK;
}

/* SYSTem:BACKlight? */
static scpi_result_t Sys_BacklightQ(scpi_t *context) {
    SCPI_ResultBool(context, backlightEnable != off);
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

/* SYSTem:POWer:DOWN — trigger power-down (forward-declared in PennerScale.cpp) */
extern void powerDown(void);
static scpi_result_t Sys_PowerDown(scpi_t *context) {
    (void) context;
    powerDown();
    return SCPI_RES_OK;  /* may not be reached */
}

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
    { .pattern = "MEASure:WEIGht:MAX?",       .callback = Meas_WeightMaxQ, },

    /* Configuration */
    { .pattern = "CONFigure:UNIT",   .callback = Conf_Unit, },
    { .pattern = "CONFigure:UNIT?",  .callback = Conf_UnitQ, },
    { .pattern = "CONFigure:TARE",   .callback = Conf_Tare, },
    { .pattern = "CONFigure:TARE?",  .callback = Conf_TareQ, },
    { .pattern = "CONFigure:ZERO",   .callback = Conf_Zero, },

    /* Calibration (read-only) */
    { .pattern = "CALibration:VALue?",  .callback = Cal_ValueQ, },
    { .pattern = "CALibration:ZERO?",   .callback = Cal_ZeroQ, },
    { .pattern = "CALibration:WEIGht?", .callback = Cal_WeightQ, },
    { .pattern = "CALibration:UNIT?",   .callback = Cal_UnitQ, },

    /* System */
    { .pattern = "SYSTem:BACKlight",              .callback = Sys_Backlight, },
    { .pattern = "SYSTem:BACKlight?",             .callback = Sys_BacklightQ, },
    { .pattern = "SYSTem:POWer:VOLTage:BATTery?", .callback = Sys_VbattQ, },
    { .pattern = "SYSTem:POWer:VOLTage:SUPPly?",  .callback = Sys_VsuppQ, },
    { .pattern = "SYSTem:POWer:DOWN",              .callback = Sys_PowerDown, },

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

    uint8_t buf[64];
    for (;;) {
        int avail = Serial.available();
        if (avail > 0) {
            int toRead = (avail > (int) sizeof(buf)) ? (int) sizeof(buf) : avail;
            int n = Serial.readBytes(buf, toRead);
            if (n > 0) {
                SCPI_Input(&scpi_context, (const char *) buf, n);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
