/* Tacuna Scale */

// Includes
#include <Arduino.h>
#include <U8g2lib.h>
#include <PRDC_AD7193.h>
#include <EEPROM.h>
#include "RunningAverage.h"
#include "appconfig.h"
#include "scpi_interface.h"
#include <esp_wifi.h>
#include <esp_bt.h>

// Library defines
// FreeRTOS
#define ARDUINO_RUNNING_CORE 1
// U8g2lib
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

// Splash logo selector
#define SPLASH_LOGO_NONE   0
#define SPLASH_LOGO_PENNER 1
#define SPLASH_LOGO_TACUNA 2
#define SPLASH_LOGO SPLASH_LOGO_TACUNA  // Select active logo

#if SPLASH_LOGO == SPLASH_LOGO_PENNER
#include "logo_penner.h"
#elif SPLASH_LOGO == SPLASH_LOGO_TACUNA
#include "logo_tacuna.h"
#endif

// Pin defines
// LCD
#define LCD_RST 15
#define LCD_A0 16
#define LCD_CS 17
// LCD_BACKLIGHT defined in appconfig.h

// User
#define PWR_ZERO_BTN 19
#define UNIT_BTN 20
#define BKL_UP_BTN 21

// Power
#define V3V3_EN 2
#define VIN_LVL_EN 4
#define VIN_LVL 5       //  10k & 3.3k Vdiv => 1/0.248
#define V5_A_LVL 6
#define V5_A_EN 7
// V3V3_PG and V5_A_PG defined in appconfig.h
// Battery level macro definition
#define BATV_TO_BAR(V) ((V) * 4.8f - 26.4f)


// Ext ADC
#define SCLK 12
#define MISO 13
#define MOSI 11
#define EXT_ADC_CS 10
#define NO_ACTIVITY_THRESHOLD_MS 300000  // 5min power-down timeout
#define NO_ACTIVITY_WEIGHT_RANGE_LB 1   // Power-down deadzone (lb)
#define NO_ACTIVITY_ADC_COUNTS 500      // Power-down deadzone (raw ADC counts, uncalibrated)
#define IDLE_ACTIVITY_PCT 0.1f          // Idle wake threshold: % of full scale (calWeight or ADC range)
#define ADC_FULL_SCALE 16777216         // AD7193 24-bit full scale (2^24)
#define EXT_ADC_RATE 150  // 150 = 4Hz (based on settling time), 120 = 5Hz, 100=6Hz, 85=7Hz.  
#define EXT_ANALOG_SETTLING_TIME (EXT_ADC_RATE * 1.6676f)
#define EXT_ANALOG_READ_TASK_DELAY (EXT_ANALOG_SETTLING_TIME + 2)  // Call the read task a little late to catch the ADC just after conversion
#define EXT_ADC_AVG_SAMPLE_NUM 5

// Config Switch
#define SW_1 8
#define SW_2 9

// Constants
#define V_3_3 3.3f
#define INT_ADC_M_VAL 0.995f
#define INT_ADC_B_VAL 0.2413f
const float PWR_VIN_LVL_COUNTS_TO_V = 1/((4096/V_3_3)*0.248); // Multiply counts by this scalar to get Volts. => 1/((4096/3.3)*0.248)
const float PWR_VIN_LVL_VDIV_SCLR = (1/0.248); // Multiply ADC Volts by this scalar to get real Volts. => (1/0.248)
const float  PWR_5V_LVL_COUNTS_TO_V = 1/((4096/V_3_3)*0.5); // Multiply counts by this scalar to get Volts. => 1/((4096/3.3)*0.5)
const float  PWR_5V_LVL_VDIV_SCLR = (1/0.5); // // Multiply ADC Volts by this scalar to get real Volts. => (1/0.5)
#define SPI_FREQ 20000000
#define INT_ADC_TASK_DELAY 5000

// UI
#define LONG_PRESS_LOOP_DELAY 10 // 10ms 
#define LONG_PRESS_COUNT_THRESHOLD 150 // multiples of LONG_PRESS_LOOP_DELAY (ex. for LONG_PRESS_DELAY = 10ms (100 counts = 1s)
#define DEBOUNCE_TIME 50
// SCALE_CAP defined in appconfig.h
#define SCALE_CAP_UNIT lb
#define MIN_CAL_VAL 25
#define MAX_CAL_VAL 500
#define DEFAULT_UNIT lb
#define USER_MSG_Y_POS 23
#define USER_MSG_Y_LINE_HEIGHT 10

// LCD weight reading formatting constants
#define WVAL_X_POS 128 // Reading is right-aligned to full display width (unit moved to line below)
#define WVAL_Y_POS 43
#define WVAL_DEC_PLS 1
#define WVAL_PREC 1
#define UNIT_Y_POS 61  // Unit at bottom of display, right-justified (3px margin for descenders)
#define DEC_PT_W_PX 5  // Actual width of the decimal point in px
#define DEC_PT_S_PX 7  // Space generated between numbers to allow for decimal point in px (usually ~1.5x dec pt width)
#define WVAL_FONT u8g2_font_inb21_mn
#define UNIT_FONT u8g2_font_7x13B_mf  // Bold, full Unicode (includes middle dot ·)
#define MSG_FONT u8g2_font_6x12_m_symbols
#define FW_FONT u8g2_font_4x6_mr
#define LCD_CONTRAST 60

// LCD battery indicator formatting constants
#define BAT_IND_WIDTH 14
#define BAT_IND_HEIGHT 6
#define BAT_BUMP_HEIGHT 2
#define BAT_BUMP_WIDTH 2
#define BAT_IND_Y_POS 4  // Top-right corner of display (3px margin to match unit bottom)
#define BAT_IND_X_POS (u8g2.getDisplayWidth() - BAT_IND_WIDTH)

// FreeRTOS constants
//  Valid CPU speeds: 240, 160, 80 (all XTAL types), 40, 20, 10 (40MHz XTAL only)
#define CPU_SPEED 80  // 80 MHz. APB=80MHz, max SPI=40MHz.
#define IDLE_MODE_THRESHOLD_MS 10000  // Seconds of stable weight before entering idle mode

// U8g2 Contructor (Frame Buffer) — Hardware SPI for ~2ms frame transfer vs ~73ms software
U8G2_ST7567_ENH_DG128064I_F_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/ LCD_CS, /* dc=*/ LCD_A0, /* reset=*/ LCD_RST);

// AD7193 Constructor
PRDC_AD7193 AD7193;

// Forward declarations - Tasks
void TaskIntAnalogRead( void *pvParameters );
void TaskExtAnalogRead( void *pvParameters );
void TaskPowerZeroButton( void *pvParameters );
void TaskUnitButton( void *pvParameters );
void TaskBKLButton( void *pvParameters );
void TaskUI( void *pvParameters );

// Forward declarations - Helper functions
void UpdateWeightReadingLCD();
uint8_t convertBattVtoBarPx(float battVolts);
#if SPLASH_LOGO
void drawLogo(void);
#endif
void doCalibration();
void sendBufferSPISafe(void);
void powerDown(void);
float removeNegativeSignFromZero(float weightValue);
float analogReadVoltage(byte pin);
float unitConversionFactor(e_unitVal from, e_unitVal to);

SemaphoreHandle_t SPImutex = xSemaphoreCreateMutex();
portMUX_TYPE measMux = portMUX_INITIALIZER_UNLOCKED;  // Spinlock for measurement data shared with SCPI

// EEPROM address helpers — map channel index to EEPROM address
static int eepromAddrCalValue(int ch)  { return ch ? EEPROM_ADDR_CH1_CAL_VALUE  : EEPROM_ADDR_CAL_VALUE; }
static int eepromAddrZeroValue(int ch) { return ch ? EEPROM_ADDR_CH1_ZERO_VALUE : EEPROM_ADDR_ZERO_VALUE; }
static int eepromAddrCalUnit(int ch)   { return ch ? EEPROM_ADDR_CH1_CAL_UNIT   : EEPROM_ADDR_CAL_UNIT; }
static int eepromAddrCalWeight(int ch) { return ch ? EEPROM_ADDR_CH1_CAL_WEIGHT : EEPROM_ADDR_CAL_WEIGHT; }

// Global variables — per-channel arrays (index 0 = Ch0, 1 = Ch1)
RunningAverage extADCRunAV[NUM_CHANNELS] = {
    RunningAverage(EXT_ADC_AVG_SAMPLE_NUM),
    RunningAverage(EXT_ADC_AVG_SAMPLE_NUM)
};
int32_t extADCResultCh[NUM_CHANNELS] = {0, 0};
int32_t extADCResult = 0;  // combined raw ADC (Ch0+Ch1 or Ch0 only, for power-down detection)
float extADCweight[NUM_CHANNELS] = {0.0f, 0.0f};
float extADCweightMax[NUM_CHANNELS] = {0.0f, 0.0f};
float vinVolts = 0.0;
float v5vVolts = 0.0;

bool configSwitch1 = 0;
bool configSwitch2 = 0;
e_displayMode displayMode = DISP_MODE_DEFAULT;

e_backlightEnable backlightEnable = off;
uint8_t backlightPWM = BACKLIGHT_PWM_DEFAULT;  // 0-100 percent
volatile bool scpiEchoEnable   = true;   // default ON
volatile bool scpiPromptEnable = true;   // default ON
// setting PWM properties
const uint32_t backlightPWMfreq = 5000;
const uint8_t backlightPWMres = 8;

static inline uint8_t pwmPercentToDuty(uint8_t pct) {
    return (uint8_t)((uint16_t)pct * 255 / 100);
}

e_unitVal unitVal[NUM_CHANNELS] = {DEFAULT_UNIT, DEFAULT_UNIT};

float calValue[NUM_CHANNELS] = {7168.220215f, 7168.220215f};
int32_t zeroValue[NUM_CHANNELS] = {0, 0};
float tareValue[NUM_CHANNELS] = {0.0f, 0.0f};
uint32_t calWeight[NUM_CHANNELS] = {MIN_CAL_VAL, MIN_CAL_VAL};
e_unitVal calUnit[NUM_CHANNELS] = {DEFAULT_UNIT, DEFAULT_UNIT};

extern const float kgtolbScalar = 2.20462;
extern const float kgtoNScalar  = 9.80665;
extern const float NmtolbftScalar = 0.737562;
const char* const unitAbbr[] = {"kg", "lb", "N", "N\xb7m", "lb\xb7""ft"};
float stabThreshold[NUM_CHANNELS] = {STAB_THRESH_DEFAULT, STAB_THRESH_DEFAULT};
float overloadCapacity[NUM_CHANNELS] = {OVER_CAP_DEFAULT, OVER_CAP_DEFAULT};
bool adaptiveFilterEnable[NUM_CHANNELS] = {ADAPT_FILTER_DEFAULT, ADAPT_FILTER_DEFAULT};
float adaptiveFilterPct[NUM_CHANNELS] = {ADAPT_THRESH_DEFAULT, ADAPT_THRESH_DEFAULT};
uint32_t adaptiveFilterTimeUs[NUM_CHANNELS] = {ADAPT_TIME_DEFAULT, ADAPT_TIME_DEFAULT};
bool adcInvert[NUM_CHANNELS] = {false, false};
bool updateLCDWeight = true;
volatile bool newWeightReady = false;
bool noActivityPowerDownFlag = false;
volatile uint32_t cpuIdleMode = 0;      // 1 when in idle mode, 32-bit for atomic ops
volatile uint32_t msAtLastIdleActivity; // Idle timer — reset by any stimulus (ADC, button, SCPI)

/* EEPROM addresses are defined as macros in appconfig.h */

enum e_buttonStat {no_press = 0, is_pressed = 1, short_press = 2, long_press = 3};
e_buttonStat powerButtonStat = no_press;  // Used to track immediate button status
e_buttonStat unitButtonStat = no_press;  // Used to track immediate button status
e_buttonStat bklButtonStat = no_press;  // Used to track immediate button status

enum e_buttonFlag {no_press_flag = 0, short_press_flag = 1, long_press_flag = 2};
e_buttonFlag powerButtonFlag = no_press_flag;  // Used to alert other tasks of a button event - cleared by UI task
e_buttonFlag unitButtonFlag = no_press_flag; // Used to alert other tasks of a button event - cleared by UI task
e_buttonFlag bklButtonFlag = no_press_flag; // Used to alert other tasks of a button event - cleared by UI task

TaskHandle_t xHandleTaskExtAnalogRead = NULL;
TaskHandle_t xHandleTaskIntAnalogRead = NULL;
TaskHandle_t xHandleTaskPowerZeroButton = NULL;
TaskHandle_t xHandleTaskUnitButton = NULL;
TaskHandle_t xHandleTaskBKLButton = NULL;
TaskHandle_t xHandleTaskUI = NULL;
TaskHandle_t xHandleTaskSCPI = NULL;

// Wake from idle mode (called on button press, ADC activity, or SCPI input)
void wakeFromIdleMode() {
    msAtLastIdleActivity = millis();
    if (cpuIdleMode) {
        cpuIdleMode = 0;
        DBG_PRINTLN("Activity detected - exiting idle mode");
    }
}

// The setup function runs once when you press reset or power on the board.
void setup() {
  // Load cell init
  float EEPROMcalValue; // calibration value from EEPROM
  uint32_t EEPROMZeroValue; // zero value from EEPROM
  e_backlightEnable EEPROMbacklightEnable;
  e_unitVal EEPROMunitVal;
  uint32_t EEPROMcalWeight;
  e_unitVal EEPROMcalUnit;
  float EEPROMextADCweightMax;

  setCpuFrequencyMhz(CPU_SPEED);

  // Fully deinit unused radios to save power and free memory
  esp_err_t err;
  err = esp_wifi_stop();        if (err != ESP_OK) DBG_PRINTF("wifi stop: %s\n", esp_err_to_name(err));
  err = esp_wifi_deinit();      if (err != ESP_OK) DBG_PRINTF("wifi deinit: %s\n", esp_err_to_name(err));
  err = esp_bt_controller_disable(); if (err != ESP_OK) DBG_PRINTF("bt disable: %s\n", esp_err_to_name(err));
  err = esp_bt_controller_deinit();  if (err != ESP_OK) DBG_PRINTF("bt deinit: %s\n", esp_err_to_name(err));

#if SCPI_DEBUG
  dbg_log_init();
#endif

  // Initialize serial communication at 115200 bits per second:
  // Explicitly set RX=44, TX=43 (UART0 default pins) to ensure GPIO matrix is configured
  Serial.begin(115200, SERIAL_8N1, 44, 43);
  delay(500);

  // Enable 3.3V 
  pinMode(V3V3_EN, OUTPUT);
  digitalWrite(V3V3_EN, HIGH);

  // Enable 5V_A
  pinMode(V5_A_EN, OUTPUT);
  digitalWrite(V5_A_EN, HIGH);

  // Power good inputs
  pinMode(V3V3_PG, INPUT);
  pinMode(V5_A_PG, INPUT);

  // Disable LCD backlight
  pinMode(LCD_BACKLIGHT, OUTPUT);
  digitalWrite(LCD_BACKLIGHT, LOW);

  // Configure LED PWM
  ledcAttach(LCD_BACKLIGHT, backlightPWMfreq, backlightPWMres);
  ledcWrite(LCD_BACKLIGHT, 0);

  // Set external ADC CS inactive
  pinMode(EXT_ADC_CS, OUTPUT);
  digitalWrite(EXT_ADC_CS, HIGH);

  // Set LCD CS inactive
  pinMode(LCD_CS, OUTPUT);
  digitalWrite(LCD_CS, HIGH);

  // User buttons
  pinMode(PWR_ZERO_BTN, INPUT);
  pinMode(UNIT_BTN, INPUT);
  pinMode(BKL_UP_BTN, INPUT);

  // Set SPI directions
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCLK, OUTPUT);
  
  // Set internal ADC input
  pinMode(VIN_LVL, INPUT);
  pinMode(V5_A_LVL, INPUT);
  pinMode(VIN_LVL_EN, OUTPUT);
  digitalWrite(VIN_LVL_EN, LOW);

  // Set config switch inputs
  pinMode(SW_1, INPUT);
  pinMode(SW_2, INPUT);
  configSwitch1 = digitalRead(SW_1);
  configSwitch2 = digitalRead(SW_2);

  EEPROM.begin(EEPROM_SIZE_BYTES);

  // --- CH0 EEPROM load (existing addresses) ---
  EEPROM.get(EEPROM_ADDR_CAL_VALUE, EEPROMcalValue);
  EEPROM.get(EEPROM_ADDR_ZERO_VALUE, EEPROMZeroValue);
  EEPROM.get(EEPROM_ADDR_BACKLIGHT, EEPROMbacklightEnable);
  EEPROM.get(EEPROM_ADDR_UNIT_VAL, EEPROMunitVal);
  EEPROM.get(EEPROM_ADDR_CAL_WEIGHT, EEPROMcalWeight);
  EEPROM.get(EEPROM_ADDR_CAL_UNIT, EEPROMcalUnit);
  EEPROM.get(EEPROM_ADDR_WEIGHT_MAX, EEPROMextADCweightMax);

  // Validate EEPROM values; use compile-time defaults for any uninitialized fields.
  // Track whether any field needed fixing so we can write defaults back.
  bool eepromDirty = false;

  // If calValue is a real number, that means we've calibrated the unit and calValue and zeroValue are legitimate values
  if ((!isnan(EEPROMcalValue)) && (EEPROMcalValue>0))
  {
    calValue[0] = EEPROMcalValue;
    zeroValue[0] = EEPROMZeroValue;
  } else { eepromDirty = true; }
  if (EEPROMbacklightEnable >= 0 && EEPROMbacklightEnable <= 2) {
    backlightEnable = EEPROMbacklightEnable;
  } else { eepromDirty = true; }
  if (EEPROMunitVal >= 0 && EEPROMunitVal < UNIT_VAL_COUNT) {
    unitVal[0] = EEPROMunitVal;
  } else { eepromDirty = true; }
  if (EEPROMcalWeight != 4294967295) {
    calWeight[0] = EEPROMcalWeight;
  } else { eepromDirty = true; }
  if (EEPROMcalUnit >= 0 && EEPROMcalUnit < UNIT_VAL_COUNT)
  {
    calUnit[0] = EEPROMcalUnit;
  }
  else
  {
    calUnit[0] = unitVal[0];
    eepromDirty = true;
  }
  if (!isnan(EEPROMextADCweightMax)) {
    extADCweightMax[0] = EEPROMextADCweightMax;
  } else { eepromDirty = true; }

  uint8_t EEPROMbacklightPWM;
  EEPROM.get(EEPROM_ADDR_BACKLIGHT_PWM, EEPROMbacklightPWM);
  if (EEPROMbacklightPWM <= 100) {
    backlightPWM = EEPROMbacklightPWM;
  } else { eepromDirty = true; }

  uint8_t EEPROMecho;
  EEPROM.get(EEPROM_ADDR_ECHO, EEPROMecho);
  if (EEPROMecho <= 1) {
    scpiEchoEnable = (bool) EEPROMecho;
  } else { eepromDirty = true; }

  uint8_t EEPROMprompt;
  EEPROM.get(EEPROM_ADDR_PROMPT, EEPROMprompt);
  if (EEPROMprompt <= 1) {
    scpiPromptEnable = (bool) EEPROMprompt;
  } else { eepromDirty = true; }

  float EEPROMstabThresh;
  EEPROM.get(EEPROM_ADDR_STAB_THRESH, EEPROMstabThresh);
  if (!isnan(EEPROMstabThresh) && EEPROMstabThresh > 0) {
    stabThreshold[0] = EEPROMstabThresh;
  } else { eepromDirty = true; }

  float EEPROMoverCap;
  EEPROM.get(EEPROM_ADDR_OVER_CAP, EEPROMoverCap);
  if (!isnan(EEPROMoverCap) && EEPROMoverCap > 0) {
    overloadCapacity[0] = EEPROMoverCap;
  } else { eepromDirty = true; }

  uint8_t EEPROMadaptEnable;
  EEPROM.get(EEPROM_ADDR_ADAPT_ENABLE, EEPROMadaptEnable);
  if (EEPROMadaptEnable <= 1) {
    adaptiveFilterEnable[0] = (bool) EEPROMadaptEnable;
  } else { eepromDirty = true; }

  float EEPROMadaptThresh;
  EEPROM.get(EEPROM_ADDR_ADAPT_THRESH, EEPROMadaptThresh);
  if (!isnan(EEPROMadaptThresh) && EEPROMadaptThresh > 0) {
    adaptiveFilterPct[0] = EEPROMadaptThresh;
  } else { eepromDirty = true; }

  uint32_t EEPROMadaptTime;
  EEPROM.get(EEPROM_ADDR_ADAPT_TIME, EEPROMadaptTime);
  if (EEPROMadaptTime >= 1 && EEPROMadaptTime <= 60000000UL) {
    adaptiveFilterTimeUs[0] = EEPROMadaptTime;
  } else { eepromDirty = true; }

  uint8_t EEPROMadcInvert;
  EEPROM.get(EEPROM_ADDR_ADC_INVERT, EEPROMadcInvert);
  if (EEPROMadcInvert <= 1) {
    adcInvert[0] = (bool) EEPROMadcInvert;
  } else { eepromDirty = true; }

  // Write validated CH0 defaults back to EEPROM for any uninitialized fields
  if (eepromDirty) {
    EEPROM.put(EEPROM_ADDR_CAL_VALUE, calValue[0]);
    EEPROM.put(EEPROM_ADDR_ZERO_VALUE, zeroValue[0]);
    EEPROM.put(EEPROM_ADDR_BACKLIGHT, backlightEnable);
    EEPROM.put(EEPROM_ADDR_UNIT_VAL, unitVal[0]);
    EEPROM.put(EEPROM_ADDR_CAL_WEIGHT, calWeight[0]);
    EEPROM.put(EEPROM_ADDR_CAL_UNIT, calUnit[0]);
    EEPROM.put(EEPROM_ADDR_WEIGHT_MAX, extADCweightMax[0]);
    EEPROM.put(EEPROM_ADDR_BACKLIGHT_PWM, backlightPWM);
    EEPROM.put(EEPROM_ADDR_ECHO, (uint8_t) scpiEchoEnable);
    EEPROM.put(EEPROM_ADDR_PROMPT, (uint8_t) scpiPromptEnable);
    EEPROM.put(EEPROM_ADDR_STAB_THRESH, stabThreshold[0]);
    EEPROM.put(EEPROM_ADDR_OVER_CAP, overloadCapacity[0]);
    EEPROM.put(EEPROM_ADDR_ADAPT_ENABLE, (uint8_t) adaptiveFilterEnable[0]);
    EEPROM.put(EEPROM_ADDR_ADAPT_THRESH, adaptiveFilterPct[0]);
    EEPROM.put(EEPROM_ADDR_ADAPT_TIME, adaptiveFilterTimeUs[0]);
    EEPROM.put(EEPROM_ADDR_ADC_INVERT, (uint8_t) adcInvert[0]);
    EEPROM.commit();
    DBG_PRINTLN("EEPROM: initialized unset CH0 fields with defaults");
  }

  // --- CH1 EEPROM load ---
  bool ch1Dirty = false;
  float ee_ch1CalValue;    EEPROM.get(EEPROM_ADDR_CH1_CAL_VALUE, ee_ch1CalValue);
  int32_t ee_ch1ZeroValue; EEPROM.get(EEPROM_ADDR_CH1_ZERO_VALUE, ee_ch1ZeroValue);
  e_unitVal ee_ch1UnitVal; EEPROM.get(EEPROM_ADDR_CH1_UNIT_VAL, ee_ch1UnitVal);
  uint32_t ee_ch1CalWt;    EEPROM.get(EEPROM_ADDR_CH1_CAL_WEIGHT, ee_ch1CalWt);
  e_unitVal ee_ch1CalUnit; EEPROM.get(EEPROM_ADDR_CH1_CAL_UNIT, ee_ch1CalUnit);
  float ee_ch1WtMax;       EEPROM.get(EEPROM_ADDR_CH1_WEIGHT_MAX, ee_ch1WtMax);
  float ee_ch1OverCap;     EEPROM.get(EEPROM_ADDR_CH1_OVER_CAP, ee_ch1OverCap);

  if (!isnan(ee_ch1CalValue) && ee_ch1CalValue > 0) {
    calValue[1] = ee_ch1CalValue;
    zeroValue[1] = ee_ch1ZeroValue;
  } else { ch1Dirty = true; }
  if (ee_ch1UnitVal >= 0 && ee_ch1UnitVal < UNIT_VAL_COUNT) {
    unitVal[1] = ee_ch1UnitVal;
  } else { ch1Dirty = true; }
  if (ee_ch1CalWt != 4294967295) {
    calWeight[1] = ee_ch1CalWt;
  } else { ch1Dirty = true; }
  if (ee_ch1CalUnit >= 0 && ee_ch1CalUnit < UNIT_VAL_COUNT) {
    calUnit[1] = ee_ch1CalUnit;
  } else { calUnit[1] = unitVal[1]; ch1Dirty = true; }
  if (!isnan(ee_ch1WtMax)) {
    extADCweightMax[1] = ee_ch1WtMax;
  } else { ch1Dirty = true; }
  if (!isnan(ee_ch1OverCap) && ee_ch1OverCap > 0) {
    overloadCapacity[1] = ee_ch1OverCap;
  } else { ch1Dirty = true; }

  float ee_ch1StabThresh;  EEPROM.get(EEPROM_ADDR_CH1_STAB_THRESH, ee_ch1StabThresh);
  if (!isnan(ee_ch1StabThresh) && ee_ch1StabThresh > 0) {
    stabThreshold[1] = ee_ch1StabThresh;
  } else { ch1Dirty = true; }

  uint8_t ee_ch1AdaptEn;   EEPROM.get(EEPROM_ADDR_CH1_ADAPT_ENABLE, ee_ch1AdaptEn);
  if (ee_ch1AdaptEn <= 1) {
    adaptiveFilterEnable[1] = (bool) ee_ch1AdaptEn;
  } else { ch1Dirty = true; }

  float ee_ch1AdaptThresh; EEPROM.get(EEPROM_ADDR_CH1_ADAPT_THRESH, ee_ch1AdaptThresh);
  if (!isnan(ee_ch1AdaptThresh) && ee_ch1AdaptThresh > 0) {
    adaptiveFilterPct[1] = ee_ch1AdaptThresh;
  } else { ch1Dirty = true; }

  uint32_t ee_ch1AdaptTime; EEPROM.get(EEPROM_ADDR_CH1_ADAPT_TIME, ee_ch1AdaptTime);
  if (ee_ch1AdaptTime >= 1 && ee_ch1AdaptTime <= 60000000UL) {
    adaptiveFilterTimeUs[1] = ee_ch1AdaptTime;
  } else { ch1Dirty = true; }

  uint8_t ee_ch1AdcInvert; EEPROM.get(EEPROM_ADDR_CH1_ADC_INVERT, ee_ch1AdcInvert);
  if (ee_ch1AdcInvert <= 1) {
    adcInvert[1] = (bool) ee_ch1AdcInvert;
  } else { ch1Dirty = true; }

  if (ch1Dirty) {
    EEPROM.put(EEPROM_ADDR_CH1_CAL_VALUE, calValue[1]);
    EEPROM.put(EEPROM_ADDR_CH1_ZERO_VALUE, zeroValue[1]);
    EEPROM.put(EEPROM_ADDR_CH1_UNIT_VAL, unitVal[1]);
    EEPROM.put(EEPROM_ADDR_CH1_CAL_WEIGHT, calWeight[1]);
    EEPROM.put(EEPROM_ADDR_CH1_CAL_UNIT, calUnit[1]);
    EEPROM.put(EEPROM_ADDR_CH1_WEIGHT_MAX, extADCweightMax[1]);
    EEPROM.put(EEPROM_ADDR_CH1_OVER_CAP, overloadCapacity[1]);
    EEPROM.put(EEPROM_ADDR_CH1_STAB_THRESH, stabThreshold[1]);
    EEPROM.put(EEPROM_ADDR_CH1_ADAPT_ENABLE, (uint8_t) adaptiveFilterEnable[1]);
    EEPROM.put(EEPROM_ADDR_CH1_ADAPT_THRESH, adaptiveFilterPct[1]);
    EEPROM.put(EEPROM_ADDR_CH1_ADAPT_TIME, adaptiveFilterTimeUs[1]);
    EEPROM.put(EEPROM_ADDR_CH1_ADC_INVERT, (uint8_t) adcInvert[1]);
    EEPROM.commit();
    DBG_PRINTLN("EEPROM: initialized unset CH1 fields with defaults");
  }

  // --- Display mode EEPROM load ---
  uint8_t ee_dispMode;
  EEPROM.get(EEPROM_ADDR_DISP_MODE, ee_dispMode);
  if (ee_dispMode < DISP_MODE_COUNT) {
    displayMode = (e_displayMode) ee_dispMode;
  } else {
    displayMode = DISP_MODE_DEFAULT;
    EEPROM.put(EEPROM_ADDR_DISP_MODE, (uint8_t) displayMode);
    EEPROM.commit();
  }

  ledcWrite(LCD_BACKLIGHT, pwmPercentToDuty(backlightPWM) * (backlightEnable != off));

  // Initialize shared SPI bus once (FSPI/SPI2 IOMUX pins — optimal for HW SPI)
  SPI.begin(SCLK, MISO, MOSI, -1);  // No automatic SS — CS managed per-device

  u8g2.setBusClock(4000000); // 4 MHz — ST7567 supports up to 20 MHz
  // Start LCD
  u8g2.begin();
  // Set contrast
  u8g2.setContrast(LCD_CONTRAST);
  // Display logo
  u8g2.clearBuffer();
#if SPLASH_LOGO
  drawLogo();
#endif
  u8g2.setFont(FW_FONT);
  u8g2.setCursor(101, 55);
  u8g2.print("v");
  u8g2.print(FW_VER);

  u8g2.sendBuffer();

  delay(750); // Logo display and serial port ready delay

  DBG_PRINTF("TacunaScale FW: %s\n\r", FW_VER);
  DBG_PRINTF("Config Switch1: %d\n\r", configSwitch1);
  DBG_PRINTF("Config Switch2: %d\n\r", configSwitch2);

  for (int c = 0; c < NUM_CHANNELS; c++) {
    DBG_PRINTF("CH%d calVal: %f  zeroVal: %d  unit: %d (%s)  calWt: %u  calUnit: %d (%s)  max: %f\n\r",
               c, calValue[c], zeroValue[c], unitVal[c], unitAbbr[unitVal[c]],
               calWeight[c], calUnit[c], unitAbbr[calUnit[c]], extADCweightMax[c]);
  }
  DBG_PRINTF("backlightEnable: %d  backlightPWM: %d%%\n\r", backlightEnable, backlightPWM);

  // Initialize external ADC

  for (int c = 0; c < NUM_CHANNELS; c++) extADCRunAV[c].clear();

  // SPI bus already initialized above — configure ADC SPI settings
  AD7193.setSPIFrequency(SPI_FREQ);
  AD7193.setSPI(SPI);
  AD7193.begin(); // ID check returns false — hardware is AD7192, library expects AD7193 — but init still works
  {
    AD7193.setClockMode(AD7193_CLK_INT);
    AD7193.setRate(EXT_ADC_RATE);
    AD7193.setFilter(AD7193_MODE_SINC4);
    AD7193.enableNotchFilter(true);
    AD7193.enableChop(true);
    AD7193.enableBuffer(true);
    AD7193.rangeSetup(0, AD7193_CONF_GAIN_128);
    AD7193.setBPDSW(true);
    AD7193.printAllRegisters();
    DBG_PRINTLN(F("ADC Initialized."));
    AD7193.channelSelect(AD7193_CH_0);

    delay(EXT_ANALOG_READ_TASK_DELAY);
    extADCResultCh[0] = AD7193.singleConversion();

    DBG_PRINTF("ADC Ch0: %d\n", extADCResultCh[0]);
    AD7193.channelSelect(AD7193_CH_1);
    delay(EXT_ANALOG_READ_TASK_DELAY);
    extADCResultCh[1] = AD7193.singleConversion();
    DBG_PRINTF("ADC Ch1: %d\n", extADCResultCh[1]);

    AD7193.channelSelect(AD7193_CH_0);

    if (adcInvert[0]) { extADCResultCh[0] = -extADCResultCh[0]; }
    if (adcInvert[1]) { extADCResultCh[1] = -extADCResultCh[1]; }
    extADCResult = extADCResultCh[0] + extADCResultCh[1];

    // Auto-tare both channels at boot
    for (int c = 0; c < NUM_CHANNELS; c++) {
      tareValue[c] = (calValue[c] != 0.0f) ? (extADCResultCh[c] - zeroValue[c]) / calValue[c] : 0.0f;
      DBG_PRINTF("CH%d Tare Value: %f\n", c, tareValue[c]);
    }
  }
  


  // u8g2.clearBuffer();
  // u8g2.sendBuffer();

  // Allow user to release button before proceeding
  while (digitalRead(PWR_ZERO_BTN))
  {
    delay(100);
  }

  // Setup SPI mutex
  assert(SPImutex);

  // Set up tasks
      xTaskCreatePinnedToCore(
    TaskIntAnalogRead
    ,  "Analog Read"
    ,  2048  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  1  // Priority
    ,  &xHandleTaskIntAnalogRead
    ,  ARDUINO_RUNNING_CORE // Core on which the task will run
    );

  xTaskCreatePinnedToCore(
    TaskExtAnalogRead
    ,  "Ext ADC Task" // A name just for humans
    ,  4096        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  NULL // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  3  // Priority
    ,  &xHandleTaskExtAnalogRead
    ,  ARDUINO_RUNNING_CORE
    );

    xTaskCreatePinnedToCore(
    TaskPowerZeroButton
    ,  "Check Power/Zero Button"
    ,  2048  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  3  // Priority — same as ADC for responsive input
    ,  &xHandleTaskPowerZeroButton // With task handle we will be able to manipulate with this task.
    ,  ARDUINO_RUNNING_CORE // Core on which the task will run
    );

    xTaskCreatePinnedToCore(
    TaskUnitButton
    ,  "Check Unit Button"
    ,  2048  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  3  // Priority — same as ADC for responsive input
    ,  &xHandleTaskUnitButton // With task handle we will be able to manipulate with this task.
    ,  ARDUINO_RUNNING_CORE // Core on which the task will run
    );

    xTaskCreatePinnedToCore(
    TaskBKLButton
    ,  "Check Aux Button"
    ,  2048  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  3  // Priority — same as ADC for responsive input
    ,  &xHandleTaskBKLButton // With task handle we will be able to manipulate with this task.
    ,  ARDUINO_RUNNING_CORE // Core on which the task will run
    );

    xTaskCreatePinnedToCore(
    TaskUI
    ,  "UI Task"
    ,  4096  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  2  // Priority — LCD + events, round-robins with SCPI
    ,  &xHandleTaskUI // With task handle we will be able to manipulate with this task.
    ,  ARDUINO_RUNNING_CORE // Core on which the task will run
    );

    xTaskCreatePinnedToCore(
    TaskSCPI
    ,  "SCPI Task"
    ,  4096  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  2  // Priority — round-robins with UI at equal priority
    ,  &xHandleTaskSCPI
    ,  ARDUINO_RUNNING_CORE // All tasks on same core now that CPU starvation is fixed
    );

  DBG_PRINTF("Scale initialized.\n");
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop(){
  vTaskDelay(portMAX_DELAY);  // Nothing to do — yield forever
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskUI(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount ();
  ( void ) pvParameters;
  for (;;)
  {
    // Check power button flag
    if (powerButtonFlag == short_press_flag)
    {
      // Do zero routine — tare all channels
      DBG_PRINTF("Zero scale flag set.\n");
      powerButtonFlag = no_press_flag;
      for (int c = 0; c < NUM_CHANNELS; c++) {
        tareValue[c] = (calValue[c] != 0.0f) ? (extADCResultCh[c] - zeroValue[c]) / calValue[c] : 0.0f;
        extADCRunAV[c].clear();
        DBG_PRINTF("CH%d Tare Value: %f\n", c, tareValue[c]);
      }
    }
    else if (powerButtonFlag == long_press_flag && bklButtonStat == no_press && unitButtonStat == no_press)
    {
      // Do long power button press task
      // Do power down
      DBG_PRINTF("Power down flag set.\n");
      powerButtonFlag = no_press_flag;
      powerDown();    
    }

    // Check unit button flag
    if (unitButtonFlag == short_press_flag)
    {
      // Change units — cycle all channels together
      DBG_PRINTF("Change unit flag set.\n");
      unitButtonFlag = no_press_flag;
      for (int c = 0; c < NUM_CHANNELS; c++) {
        e_unitVal oldUnit = unitVal[c];
        unitVal[c] = (e_unitVal)((unitVal[c] + 1) % UNIT_VAL_COUNT);
        float convFactor = unitConversionFactor(oldUnit, unitVal[c]);
        overloadCapacity[c] *= convFactor;
        extADCweightMax[c] *= convFactor;
        extADCRunAV[c].clear();
      }
    }
    else if (unitButtonFlag == long_press_flag)
    {
      if(bklButtonStat == is_pressed || bklButtonFlag == long_press_flag)
      {
        // We've received a press with power button reaching the long press status first.
        // If both buttons have reached long press status, execute task.
        // Otherwise come back and check if unit button has reached long press status.
        // Note this is 1 of 2 places where this can occur...
        // The other is in the power button tests above.

        if(bklButtonFlag == long_press_flag)
        {
          // Both buttons have reached long-press status.  Execute double long-press task.
          bklButtonFlag = no_press_flag;
          unitButtonFlag = no_press_flag;
          DBG_PRINTF("Calibrate flag set.\n");
          doCalibration();
        }
      }
      else
      {
        // Do unit button long press action
        unitButtonFlag = no_press_flag;
      }
    }

    // Check bkl button flag
    if (bklButtonFlag == short_press_flag)
    {
      // Do bkl button short press action and reset flag
      bklButtonFlag = no_press_flag;

      // Cycle backlight setting
      if(backlightEnable == off) backlightEnable = on;
      else if(backlightEnable == on) backlightEnable = off;
      //else if(backlightEnable == on) backlightEnable = on_motion;
      //else if(backlightEnable == on_motion) backlightEnable = off;        
      DBG_PRINTF("Backlight toggle flag set. Backlight = %d\n", backlightEnable);
      ledcWrite(LCD_BACKLIGHT, pwmPercentToDuty(backlightPWM) * (backlightEnable != off));

    }
    else if(bklButtonFlag == long_press_flag)
    {
      if(unitButtonStat == is_pressed || unitButtonFlag == long_press_flag)
      {
        // We've received a press with bkl button reaching the long press status first.
        // If both buttons have reached long press status, execute task.
        // Otherwise come back and check if unit button has reached long press status.
        // Note this is 1 of 2 places where this can occur...
        // The other is in the unit button tests above.

        if(unitButtonFlag == long_press_flag)
        {
          // Both buttons have reached long-press status.  Execute double long-press task.
          bklButtonFlag = no_press_flag;
          unitButtonFlag = no_press_flag;
          DBG_PRINTF("Calibrate flag set.\n");
          doCalibration();
        }

      }
      else
      {
        // Do bkl button long press action
        bklButtonFlag = no_press_flag;
      }
    }
    // Update LCD when new ADC data is available
    if (newWeightReady) {
      UpdateWeightReadingLCD();
      newWeightReady = false;
    }

    xTaskDelayUntil(&xLastWakeTime, 25/portTICK_PERIOD_MS);
  }
}

void TaskExtAnalogRead(void *pvParameters)
{
  ( void ) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount ();
  static uint32_t msAtLastWeightChange = millis();  // Power-down timer (reset on >1 lb change)
  static float lastExtADCweight[NUM_CHANNELS] = {0, 0};
  static float weightChangeLB = 0;
  static int32_t lastExtADCResultRaw = 0;
  // Per-channel adaptive filter state
  static uint32_t adaptiveStartUs[NUM_CHANNELS] = {0, 0};
  static bool adaptiveLastAbove[NUM_CHANNELS] = {false, false};
  static bool adaptiveTracking[NUM_CHANNELS] = {false, false};
  msAtLastIdleActivity = millis();  // Initialize idle timer

  xTaskDelayUntil(&xLastWakeTime, EXT_ANALOG_READ_TASK_DELAY/portTICK_PERIOD_MS);

  for (;;){ // A Task shall never return or exit.
    for (int c = 0; c < NUM_CHANNELS; c++) lastExtADCweight[c] = extADCweight[c];

    // --- Ch0 read (mutex held only during SPI transaction) ---
    xSemaphoreTake(SPImutex, portMAX_DELAY);
    int32_t ch0 = AD7193.singleConversion();
    AD7193.channelSelect(AD7193_CH_1);
    xSemaphoreGive(SPImutex);

    // Settling time between channel reads — mutex FREE, LCD can update here
    xTaskDelayUntil(&xLastWakeTime, EXT_ANALOG_READ_TASK_DELAY/portTICK_PERIOD_MS);

    // --- Ch1 read (mutex held only during SPI transaction) ---
    xSemaphoreTake(SPImutex, portMAX_DELAY);
    int32_t ch1 = AD7193.singleConversion();
    AD7193.channelSelect(AD7193_CH_0);
    xSemaphoreGive(SPImutex);

    // Apply per-channel polarity inversion to locals before writing globals
    if (adcInvert[0]) { ch0 = -ch0; }
    if (adcInvert[1]) { ch1 = -ch1; }
    // Update shared measurement data (synchronized with SCPI reads)
    taskENTER_CRITICAL(&measMux);

    extADCResultCh[0] = ch0;
    extADCResultCh[1] = ch1;
    extADCResult = ch0 + ch1;  // combined raw for power-down detection

    // Per-channel weight computation
    for (int c = 0; c < NUM_CHANNELS; c++) {
      extADCweight[c] = (calValue[c] != 0.0f)
          ? (extADCResultCh[c] - zeroValue[c]) / calValue[c] - tareValue[c]
          : 0.0f;
      extADCweight[c] *= unitConversionFactor(calUnit[c], unitVal[c]);
      extADCweight[c] = removeNegativeSignFromZero(extADCweight[c]);
    }

    for (int c = 0; c < NUM_CHANNELS; c++) {
      if (fabsf(extADCweight[c]) > fabsf(extADCweightMax[c]))
      {
        extADCweightMax[c] = extADCweight[c];
      }
      // Adaptive filter per channel: clear average on sustained directional change
      if (adaptiveFilterEnable[c] && extADCRunAV[c].getCount() > 0) {
        float avg = extADCRunAV[c].getAverage();
        float thresh = overloadCapacity[c] * adaptiveFilterPct[c] / 100.0f;
        float delta = extADCweight[c] - avg;
        uint32_t nowUs = micros();
        if (thresh > 0.0f && fabsf(delta) > thresh) {
          bool above = (delta > 0.0f);
          if (!adaptiveTracking[c] || above != adaptiveLastAbove[c]) {
            adaptiveStartUs[c] = nowUs;
            adaptiveLastAbove[c] = above;
            adaptiveTracking[c] = true;
          } else if ((nowUs - adaptiveStartUs[c]) >= adaptiveFilterTimeUs[c]) {
            extADCRunAV[c].clear();
            adaptiveTracking[c] = false;
          }
        } else {
          adaptiveTracking[c] = false;
        }
      } else {
        adaptiveTracking[c] = false;
      }
      extADCRunAV[c].add(extADCweight[c]);
    }
    taskEXIT_CRITICAL(&measMux);

    // Signal UI task that new weight data is available
    newWeightReady = true;

    // Compute weight and ADC changes (use ch0 for power-down/idle detection)
    int32_t adcChange = abs(extADCResult - lastExtADCResultRaw);
    lastExtADCResultRaw = extADCResult;
    weightChangeLB = abs(lastExtADCweight[0] - extADCweight[0]);
    // Convert weight change to lb for power-down/idle threshold comparison
    weightChangeLB *= unitConversionFactor(unitVal[0], lb);

    // --- Power-down activity detection (original thresholds) ---
    bool powerDownActivity = (calValue[0] != 0.0f)
      ? (weightChangeLB > NO_ACTIVITY_WEIGHT_RANGE_LB)
      : (adcChange > NO_ACTIVITY_ADC_COUNTS);
    if(powerDownActivity) {
      msAtLastWeightChange = millis();
    }
    uint32_t msSinceActivity = millis() - msAtLastWeightChange;
    if((msSinceActivity > NO_ACTIVITY_THRESHOLD_MS) && (vinVolts > 5.1))
    {
      noActivityPowerDownFlag = true;
      DBG_PRINTLN("No activity - power down.");
      powerDown();
    }
    else
    {
      noActivityPowerDownFlag = false;
    }

    // --- Idle mode activity detection (sensitive threshold: % of full scale) ---
    // Derive full-scale capacity from ADC range and calibration factor
    // (calWeight is just the reference weight, not the scale's capacity)
    float fullScaleWeight = (float)ADC_FULL_SCALE / calValue[0];  // in calUnit[0]
    float idleThresholdLB = fullScaleWeight * unitConversionFactor(calUnit[0], lb) * IDLE_ACTIVITY_PCT / 100.0f;
    int32_t idleThresholdADC = (int32_t)((float)ADC_FULL_SCALE * IDLE_ACTIVITY_PCT / 100.0f);
    bool idleActivity = (calValue[0] != 0.0f)
      ? (weightChangeLB > idleThresholdLB)
      : (adcChange > idleThresholdADC);
    if(idleActivity) {
      wakeFromIdleMode();
    }
    // Enter idle mode after IDLE_MODE_THRESHOLD_MS of no stimulus
    uint32_t msIdleAge = millis() - msAtLastIdleActivity;
    if (!cpuIdleMode && (msIdleAge > IDLE_MODE_THRESHOLD_MS))
    {
      cpuIdleMode = 1;
      DBG_PRINTLN("Entering idle mode");
    }
    xTaskDelayUntil(&xLastWakeTime, EXT_ANALOG_READ_TASK_DELAY/portTICK_PERIOD_MS);
  }
}

void TaskIntAnalogRead(void *pvParameters)
{
  ( void ) pvParameters;
  for (;;)
  {
    digitalWrite(VIN_LVL_EN, HIGH);
    vTaskDelay(10/portTICK_PERIOD_MS);  // Delay to settle LPF
    // read the input on analog pin:
    //sensorValue = analogRead(VIN_LVL)*PWR_VIN_LVL_COUNTS_TO_V*INT_ADC_M_VAL+INT_ADC_B_VAL;
    vinVolts = analogReadVoltage(VIN_LVL)*PWR_VIN_LVL_VDIV_SCLR;
    char vBuffer[6];
    //dtostrf(vinVolts,4,2, vBuffer);  // Cannot use printf with floats with small task sizes (<2048) - so do this instead
    digitalWrite(VIN_LVL_EN, LOW);
    // print out the value you read:
    //DBG_PRINTF("Vin: %s\t", vBuffer);
    //sensorValue = analogRead(V5_A_LVL)*PWR_5V_LVL_COUNTS_TO_V*INT_ADC_M_VAL+INT_ADC_B_VAL;
    v5vVolts = analogReadVoltage(V5_A_LVL)*PWR_5V_LVL_VDIV_SCLR;
    //dtostrf(v5vVolts,4,2, vBuffer);
    //DBG_PRINTF("5V_A: %s\n", vBuffer);
    vTaskDelay(INT_ADC_TASK_DELAY/portTICK_PERIOD_MS);
  }
}

void TaskPowerZeroButton(void *pvParameters)
{ 
  static uint8_t powerButtonCounter = 0;
  TickType_t xLastWakeTime = xTaskGetTickCount ();

  ( void ) pvParameters;

  // Wait here until the user releases the power button after initial power-on
  while(digitalRead(PWR_ZERO_BTN))
  {
    vTaskDelay(250/portTICK_PERIOD_MS);
  }

  for (;;)
  {
    
    // Check power/zero button
    if(digitalRead(PWR_ZERO_BTN))
    {
      if(powerButtonStat == is_pressed)
      {
        // Button is still pressed after debounce period so delay for a period of time
        while(digitalRead(PWR_ZERO_BTN) && powerButtonCounter < LONG_PRESS_COUNT_THRESHOLD)
        {
          vTaskDelay(LONG_PRESS_LOOP_DELAY/portTICK_PERIOD_MS);
          powerButtonCounter++;
        }
        if(powerButtonCounter >= LONG_PRESS_COUNT_THRESHOLD)
        {
          // Long-press detected
          powerButtonStat = long_press;
          powerButtonFlag = long_press_flag;
          DBG_PRINTF("Long power/zero button press.\n");
          // While user has button held down, don't do anything else
          while(digitalRead(PWR_ZERO_BTN))
          {
            vTaskDelay(100/portTICK_PERIOD_MS);
          }
        }
        else
        {
          // Short press detected
          powerButtonStat = short_press;
          powerButtonFlag = short_press_flag;
          DBG_PRINTF("Short power/zero button press.\n");
        }
      }
      else
      {
        // Fresh button press event detected
        powerButtonStat = is_pressed;
        wakeFromIdleMode();
      }
    }
    else
    {
      powerButtonStat = no_press;
      powerButtonCounter = 0;
    }

    xTaskDelayUntil(&xLastWakeTime, DEBOUNCE_TIME/portTICK_PERIOD_MS);
  }
}

void TaskUnitButton(void *pvParameters)
{  
  static uint8_t unitButtonCounter = 0;
  TickType_t xLastWakeTime = xTaskGetTickCount ();

  ( void ) pvParameters;

  for (;;)
  {
    
    // Check power/zero button
    if(!digitalRead(UNIT_BTN))
    {
      if(unitButtonStat == is_pressed)
      {
        // Button is still pressed after debounce period so delay for a period of time
        while((!digitalRead(UNIT_BTN)) && unitButtonCounter < LONG_PRESS_COUNT_THRESHOLD)
        {
          vTaskDelay(LONG_PRESS_LOOP_DELAY/portTICK_PERIOD_MS);
          unitButtonCounter++;
        }
        if(unitButtonCounter >= LONG_PRESS_COUNT_THRESHOLD)
        {
          // Long-press detected
          unitButtonStat = long_press;
          unitButtonFlag = long_press_flag;
          DBG_PRINTF("Long unit button press.\n");
          // While user has button held down, don't do anything else
          while(!digitalRead(UNIT_BTN))
          {
            vTaskDelay(100/portTICK_PERIOD_MS);
          }
        }
        else
        {
          // Short press detected
          unitButtonStat = short_press;
          unitButtonFlag = short_press_flag;
          DBG_PRINTF("Short unit button press.\n");
        }
      }
      else
      {
        // Fresh button press event detected
        unitButtonStat = is_pressed;
        wakeFromIdleMode();
      }
    }
    else
    {
      unitButtonStat = no_press;
      unitButtonCounter = 0;
    }
    xTaskDelayUntil(&xLastWakeTime, DEBOUNCE_TIME/portTICK_PERIOD_MS);
  }
}

void TaskBKLButton(void *pvParameters)
{  
  static uint8_t bklButtonCounter = 0;
  TickType_t xLastWakeTime = xTaskGetTickCount ();

  ( void ) pvParameters;

  for (;;)
  {
    
    // Check aux button
    if(!digitalRead(BKL_UP_BTN))
    {
      if(bklButtonStat == is_pressed)
      {
        // Button is still pressed after debounce period so delay for a period of time
        while((!digitalRead(BKL_UP_BTN)) && bklButtonCounter < LONG_PRESS_COUNT_THRESHOLD)
        {
          vTaskDelay(LONG_PRESS_LOOP_DELAY/portTICK_PERIOD_MS);
          bklButtonCounter++;
        }
        if(bklButtonCounter >= LONG_PRESS_COUNT_THRESHOLD)
        {
          // Long-press detected
          bklButtonStat = long_press;
          bklButtonFlag = long_press_flag;
          DBG_PRINTF("Long backlight button press.\n");
          // While user has button held down, don't do anything else
          while(!digitalRead(BKL_UP_BTN))
          {
            vTaskDelay(100/portTICK_PERIOD_MS);
          }
        }
        else
        {
          // Short press detected
          bklButtonStat = short_press;
          bklButtonFlag = short_press_flag;
          DBG_PRINTF("Short backlight button press.\n");
        }
      }
      else
      {
        // Fresh button press event detected
        bklButtonStat = is_pressed;
        wakeFromIdleMode();
      }
    }
    else
    {
      bklButtonStat = no_press;
      bklButtonCounter = 0;
    }
    xTaskDelayUntil(&xLastWakeTime, DEBOUNCE_TIME/portTICK_PERIOD_MS);
  }
}

// Dual-scale display fonts
#define DUAL_WVAL_FONT u8g2_font_inb16_mn   // Smaller weight font for split display
#define DUAL_UNIT_FONT u8g2_font_6x12_mr    // Smaller unit font for split display
#define DUAL_LABEL_FONT u8g2_font_5x8_mr    // Channel label font

void UpdateWeightReadingLCD()
{
  if(updateLCDWeight)
  {
    u8g2.clearBuffer();

    if (displayMode == DISP_SINGLE || displayMode == DISP_SUM) {
      // --- Single-value layout: CH0 only or CH0+CH1 summed ---
      float displayWeight;
      if (displayMode == DISP_SUM) {
        float avg0 = extADCRunAV[0].getAverage();
        float avg1 = extADCRunAV[1].getAverage();
        displayWeight = avg0 + avg1 * unitConversionFactor(unitVal[1], unitVal[0]);
      } else {
        displayWeight = extADCRunAV[0].getAverage();
      }

      u8g2.setFont(WVAL_FONT);
      String s_w = String(displayWeight, WVAL_DEC_PLS);
      u8g2.setCursor(WVAL_X_POS-u8g2.getStrWidth(s_w.substring(s_w.length()-WVAL_DEC_PLS).c_str())-DEC_PT_S_PX+((DEC_PT_S_PX-DEC_PT_W_PX)/2)-DEC_PT_W_PX-1, WVAL_Y_POS);
      u8g2.print(".");
      u8g2.setCursor(WVAL_X_POS-u8g2.getStrWidth((s_w.c_str()+1))-DEC_PT_S_PX, WVAL_Y_POS);
      u8g2.print(s_w.substring(0, s_w.length()-WVAL_DEC_PLS-1).c_str());
      u8g2.setCursor(WVAL_X_POS-u8g2.getStrWidth(s_w.substring(s_w.length()-WVAL_DEC_PLS).c_str()), WVAL_Y_POS);
      u8g2.print(s_w.substring(s_w.length()-WVAL_DEC_PLS).c_str());

      u8g2.setFont(UNIT_FONT);
      e_unitVal uSnap = unitVal[0];
      const char *uStr = ((int)uSnap >= 0 && (int)uSnap < UNIT_VAL_COUNT)
          ? unitAbbr[uSnap] : "?";
      u8g2.setCursor(u8g2.getDisplayWidth() - u8g2.getStrWidth(uStr), UNIT_Y_POS);
      u8g2.print(uStr);
    } else {
      // --- Dual channel mode: split display ---
      // Layout: Ch0 top half (y=0..31), Ch1 bottom half (y=32..63)
      for (int c = 0; c < NUM_CHANNELS; c++) {
        int yBase = c * 32;  // 0 for Ch0, 32 for Ch1
        int yBottom = yBase + 28;  // baseline: 3px above divider/display bottom

        // Unit label — right-justified (measure first to position weight)
        u8g2.setFont(DUAL_UNIT_FONT);
        e_unitVal uSnap = unitVal[c];
        const char *uStr = ((int)uSnap >= 0 && (int)uSnap < UNIT_VAL_COUNT)
            ? unitAbbr[uSnap] : "?";
        int unitWidth = u8g2.getStrWidth(uStr);
        int unitX = 128 - unitWidth;
        u8g2.setCursor(unitX, yBottom);
        u8g2.print(uStr);

        // Weight value — right-justified, 2px gap before unit
        u8g2.setFont(DUAL_WVAL_FONT);
        char wBuf[12];
        float avg = extADCRunAV[c].getAverage();
        dtostrf(avg, 7, WVAL_DEC_PLS, wBuf);
        int wWidth = u8g2.getStrWidth(wBuf);
        u8g2.setCursor(unitX - 2 - wWidth, yBottom);
        u8g2.print(wBuf);

        // Channel label — drawn last so it's not overwritten by weight font
        u8g2.setFont(DUAL_LABEL_FONT);
        char label[5];
        snprintf(label, sizeof(label), "Ch%d", c);
        u8g2.setCursor(0, yBase + 9);
        u8g2.print(label);
      }

      // Divider line between channels
      u8g2.drawHLine(0, 31, 128);
    }

    // Print battery indicator (always visible, top-right)
    u8g2.drawFrame(BAT_IND_X_POS, BAT_IND_Y_POS, BAT_IND_WIDTH - BAT_BUMP_WIDTH, BAT_IND_HEIGHT);
    u8g2.drawFrame(BAT_IND_X_POS + BAT_IND_WIDTH - BAT_BUMP_WIDTH, BAT_IND_Y_POS + BAT_IND_HEIGHT/2 - BAT_BUMP_HEIGHT/2, BAT_BUMP_WIDTH, BAT_BUMP_HEIGHT);
    u8g2.drawBox(BAT_IND_X_POS, BAT_IND_Y_POS, convertBattVtoBarPx(vinVolts), BAT_IND_HEIGHT);

    sendBufferSPISafe();
  }
}

uint8_t convertBattVtoBarPx(float battVolts)
{
  float barPx = BATV_TO_BAR(battVolts);
  if (barPx > 12) barPx = 12;
  if (barPx < 0) barPx = 0;
  return (uint8_t)barPx;
}

#if SPLASH_LOGO
void drawLogo(void)
{
  u8g2.drawXBM(LOGO_X_POS, LOGO_Y_POS, LOGO_WIDTH, LOGO_HEIGHT, splash_logo_bits);
}
#endif

void doCalibration()
{
  DBG_PRINTLN("Turning off weight update flag.");
  updateLCDWeight = false;
  vTaskDelay(500/portTICK_PERIOD_MS); // Delay to allow LCD update task to finish and read updateLCDWeight flag
  u8g2.clearBuffer();
  u8g2.setFont(MSG_FONT);
  u8g2.setCursor(0, USER_MSG_Y_POS);
  u8g2.print("--Scale Calibration--"); 
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
  u8g2.print("Press ZERO to cont.");
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT * 2);
  u8g2.print("Press UNIT to exit.");
  sendBufferSPISafe();

  // Allow user to release buttons before proceeding
  while (powerButtonFlag != no_press_flag || unitButtonFlag != no_press_flag)
  {
    vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS);
  }

  // Reset button flags
  powerButtonFlag = no_press_flag;
  unitButtonFlag = no_press_flag;

  // Wait for user to press a button
  while (powerButtonFlag == no_press_flag && unitButtonFlag == no_press_flag)
  {
    vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS);
  }

  if(unitButtonFlag != no_press_flag)
  {
    DBG_PRINTF("Calibration aborted.\n");
    // Reset button flags
    powerButtonFlag = no_press_flag;
    unitButtonFlag = no_press_flag;
    DBG_PRINTLN("Setting weight update flag.");
    updateLCDWeight = true;   
    return;
  }

  // Reset button flags
  powerButtonFlag = no_press_flag;
  unitButtonFlag = no_press_flag;  

  // Allow user to release ZERO before proceeding
  while (powerButtonFlag != no_press_flag)
  {
    vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS);
  }

  // --- Per-channel calibration: unit, weight, zero, span for each channel ---
  for (int c = 0; c < NUM_CHANNELS; c++) {

    // After CH0, offer to skip remaining channels
    if (c > 0) {
      u8g2.clearBuffer();
      u8g2.setCursor(0, USER_MSG_Y_POS);
      u8g2.printf("Calibrate CH%d?", c);
      u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
      u8g2.print("ZERO=Yes  UNIT=Skip");
      sendBufferSPISafe();
      powerButtonFlag = no_press_flag;
      unitButtonFlag = no_press_flag;
      while (powerButtonFlag == no_press_flag && unitButtonFlag == no_press_flag) {
        vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS);
      }
      bool skip = (unitButtonFlag != no_press_flag);
      powerButtonFlag = no_press_flag;
      unitButtonFlag = no_press_flag;
      if (skip) {
        DBG_PRINTF("CH%d calibration skipped by user\n", c);
        break;
      }
    }

    // --- Select calibration unit for this channel ---
    e_unitVal selCalUnit = calUnit[c];
    u8g2.clearBuffer();
    u8g2.setCursor(0, USER_MSG_Y_POS);
    u8g2.printf("--Cal CH%d: Unit--", c);
    u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
    u8g2.printf("calibration units: %s", unitAbbr[selCalUnit]);
    u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT * 2);
    u8g2.print("Press ZERO to cont.");
    sendBufferSPISafe();

    powerButtonFlag = no_press_flag;
    unitButtonFlag = no_press_flag;
    while (powerButtonFlag == no_press_flag)
    {
      if (unitButtonFlag == short_press_flag)
      {
        DBG_PRINTF("CH%d: Change cal unit flag set.\n", c);
        unitButtonFlag = no_press_flag;
        selCalUnit = (e_unitVal)((selCalUnit + 1) % UNIT_VAL_COUNT);
        u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
        u8g2.printf("calibration units: %-4s", unitAbbr[selCalUnit]);
        sendBufferSPISafe();
      }
      vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS);
    }
    calUnit[c] = selCalUnit;
    powerButtonFlag = no_press_flag;

    // --- Select calibration weight for this channel ---
    uint32_t selCalWeight = calWeight[c];
    u8g2.clearBuffer();
    u8g2.setCursor(0, USER_MSG_Y_POS);
    u8g2.printf("--Cal CH%d: Weight--", c);
    u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
    u8g2.printf("cal. weight: %5u %s", selCalWeight, unitAbbr[selCalUnit]);
    u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT * 2);
    u8g2.print("Press ZERO to cont.");
    sendBufferSPISafe();

    while (powerButtonFlag == no_press_flag)
    {
      if (bklButtonStat)
      {
        if (bklButtonStat == long_press)
        {
          if (selCalWeight < (MAX_CAL_VAL-10)) selCalWeight += 10;
          else selCalWeight = MAX_CAL_VAL;
        }
        else
        {
          if (selCalWeight < (MAX_CAL_VAL)) selCalWeight ++;
        }
        u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
        u8g2.printf("cal. weight: %5u %s", selCalWeight, unitAbbr[selCalUnit]);
        sendBufferSPISafe();
      }
      if (unitButtonStat)
      {
        if (unitButtonStat == long_press)
        {
          if (selCalWeight > (MIN_CAL_VAL + 10)) selCalWeight -= 10;
          else selCalWeight = MIN_CAL_VAL;
        }
        else
        {
          if (selCalWeight > MIN_CAL_VAL) selCalWeight --;
        }
        u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
        u8g2.printf("cal. weight: %5u %s", selCalWeight, unitAbbr[selCalUnit]);
        sendBufferSPISafe();
      }
      vTaskDelay(150/portTICK_PERIOD_MS);
    }
    calWeight[c] = selCalWeight;
    powerButtonFlag = no_press_flag;
    unitButtonFlag = no_press_flag;
    bklButtonFlag = no_press_flag;

    // --- Zero capture ---
    for (int cc = 0; cc < NUM_CHANNELS; cc++) extADCRunAV[cc].clear();

    u8g2.clearBuffer();
    u8g2.setCursor(0, USER_MSG_Y_POS);
    u8g2.printf("--Cal CH%d: Zero--", c);
    u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
    u8g2.print("Unload scale then");
    u8g2.setCursor(0, USER_MSG_Y_POS + 2 * USER_MSG_Y_LINE_HEIGHT);
    u8g2.print("press ZERO.");
    sendBufferSPISafe();
    while (powerButtonFlag == no_press_flag) { vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS); }
    powerButtonFlag = no_press_flag;
    unitButtonFlag = no_press_flag;

    u8g2.clearBuffer();
    u8g2.setCursor(0, USER_MSG_Y_POS);
    u8g2.printf("--Cal CH%d: Zero--", c);
    u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
    u8g2.print("Wait...");
    sendBufferSPISafe();
    vTaskDelay(5000/portTICK_PERIOD_MS);

    zeroValue[c] = extADCResultCh[c];

    // --- Span capture ---
    u8g2.clearBuffer();
    u8g2.setCursor(0, USER_MSG_Y_POS);
    u8g2.printf("--Cal CH%d: Span--", c);
    u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
    u8g2.printf("Load %u %s then press", calWeight[c], unitAbbr[calUnit[c]]);
    u8g2.setCursor(0, USER_MSG_Y_POS + 2 * USER_MSG_Y_LINE_HEIGHT);
    u8g2.print("ZERO.");
    sendBufferSPISafe();
    while (powerButtonFlag == no_press_flag) { vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS); }
    powerButtonFlag = no_press_flag;
    unitButtonFlag = no_press_flag;

    u8g2.clearBuffer();
    u8g2.setCursor(0, USER_MSG_Y_POS);
    u8g2.printf("--Cal CH%d: Span--", c);
    u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
    u8g2.print("Wait...");
    sendBufferSPISafe();
    vTaskDelay(5000/portTICK_PERIOD_MS);

    int32_t spanRaw;
    int32_t zeroSnap;
    taskENTER_CRITICAL(&measMux);
    spanRaw  = extADCResultCh[c];
    zeroSnap = zeroValue[c];
    taskEXIT_CRITICAL(&measMux);
    float spanCounts = (float)spanRaw - (float)zeroSnap;
    if (fabsf(spanCounts) < 1.0f || calWeight[c] == 0) {
      DBG_PRINTF("CH%d: invalid span (raw=%d zero=%d wt=%u) — skipping\n",
                 c, spanRaw, zeroSnap, calWeight[c]);
      continue;
    }
    calValue[c] = spanCounts / (float)calWeight[c];

    DBG_PRINTF("CH%d: adc=%d zero=%d calVal=%f unit=%s weight=%u\n",
               c, spanRaw, zeroSnap, calValue[c],
               unitAbbr[calUnit[c]], calWeight[c]);

    // Persist this channel immediately (safe if process interrupted)
    EEPROM.put(eepromAddrCalValue(c), calValue[c]);
    EEPROM.put(eepromAddrZeroValue(c), zeroValue[c]);
    EEPROM.put(eepromAddrCalUnit(c), calUnit[c]);
    EEPROM.put(eepromAddrCalWeight(c), calWeight[c]);
    EEPROM.commit();
  }

  // Show results
  u8g2.clearBuffer();
  u8g2.setCursor(0, USER_MSG_Y_POS);
  u8g2.print("Cal done. Press ZERO.");
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
  u8g2.printf("C0: %.2f  C1: %.2f", calValue[0], calValue[1]);
  sendBufferSPISafe();
  while (powerButtonFlag == no_press_flag) { vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS); }

  for (int c = 0; c < NUM_CHANNELS; c++) {
    extADCRunAV[c].clear();
    tareValue[c] = 0;
  }

  // Reset button flags
  powerButtonFlag = no_press_flag;
  unitButtonFlag = no_press_flag;
  DBG_PRINTLN("Setting weight update flag.");
  updateLCDWeight = true;
}

void sendBufferSPISafe(void)
{
  if (xSemaphoreTake(SPImutex, (TickType_t) 2000) == pdTRUE) // enter critical section
  {
    u8g2.sendBuffer();  // HW_SPI handles transaction + CS internally
    xSemaphoreGive(SPImutex); // exit critical section
  }
}

void powerDown(void)
{
  // Persist CH0 per-channel data
  EEPROM.put(EEPROM_ADDR_UNIT_VAL, unitVal[0]);
  EEPROM.put(EEPROM_ADDR_WEIGHT_MAX, extADCweightMax[0]);
  EEPROM.put(EEPROM_ADDR_OVER_CAP, overloadCapacity[0]);

  // Persist CH1 per-channel data
  EEPROM.put(EEPROM_ADDR_CH1_UNIT_VAL, unitVal[1]);
  EEPROM.put(EEPROM_ADDR_CH1_WEIGHT_MAX, extADCweightMax[1]);
  EEPROM.put(EEPROM_ADDR_CH1_OVER_CAP, overloadCapacity[1]);

  // Persist shared settings
  EEPROM.put(EEPROM_ADDR_BACKLIGHT, backlightEnable);
  EEPROM.put(EEPROM_ADDR_BACKLIGHT_PWM, backlightPWM);
  EEPROM.put(EEPROM_ADDR_ECHO, (uint8_t) scpiEchoEnable);
  EEPROM.put(EEPROM_ADDR_PROMPT, (uint8_t) scpiPromptEnable);
  EEPROM.put(EEPROM_ADDR_DISP_MODE, (uint8_t) displayMode);

  // Persist per-channel settings (CH0 at original addresses, CH1 at CH1_ addresses)
  EEPROM.put(EEPROM_ADDR_STAB_THRESH, stabThreshold[0]);
  EEPROM.put(EEPROM_ADDR_ADAPT_ENABLE, (uint8_t) adaptiveFilterEnable[0]);
  EEPROM.put(EEPROM_ADDR_ADAPT_THRESH, adaptiveFilterPct[0]);
  EEPROM.put(EEPROM_ADDR_ADAPT_TIME, adaptiveFilterTimeUs[0]);
  EEPROM.put(EEPROM_ADDR_ADC_INVERT, (uint8_t) adcInvert[0]);
  EEPROM.put(EEPROM_ADDR_CH1_STAB_THRESH, stabThreshold[1]);
  EEPROM.put(EEPROM_ADDR_CH1_ADAPT_ENABLE, (uint8_t) adaptiveFilterEnable[1]);
  EEPROM.put(EEPROM_ADDR_CH1_ADAPT_THRESH, adaptiveFilterPct[1]);
  EEPROM.put(EEPROM_ADDR_CH1_ADAPT_TIME, adaptiveFilterTimeUs[1]);
  EEPROM.put(EEPROM_ADDR_CH1_ADC_INVERT, (uint8_t) adcInvert[1]);

  vTaskDelay(50/portTICK_PERIOD_MS);
  EEPROM.commit();
  vTaskDelay(50/portTICK_PERIOD_MS);

  // Disable LCD backlight, clear LCD, power down
  updateLCDWeight = false;
  ledcWrite(LCD_BACKLIGHT, 0);
  u8g2.clearBuffer();
  sendBufferSPISafe();
  u8g2.setPowerSave(true);
  vTaskDelay(500/portTICK_PERIOD_MS);
  while(digitalRead(PWR_ZERO_BTN));
  digitalWrite(V3V3_EN, LOW);
  for (;;);
}

/* Return the multiplicative conversion factor from one display unit to another.
   Conversions are only meaningful within the same measurement type:
     mass:   kg ↔ lb
     torque: Nm ↔ lbft
   Cross-type changes (e.g. kg → Nm) return 1.0 (no conversion). */
float unitConversionFactor(e_unitVal from, e_unitVal to) {
  if (from == to) return 1.0f;
  // Mass conversions
  if (from == kg && to == lb) return kgtolbScalar;
  if (from == lb && to == kg) return 1.0f / kgtolbScalar;
  // Force conversions (kg ↔ N)
  if (from == kg && to == N)  return kgtoNScalar;
  if (from == N  && to == kg) return 1.0f / kgtoNScalar;
  // Force ↔ mass cross (lb ↔ N) via kg
  if (from == lb && to == N)  return (1.0f / kgtolbScalar) * kgtoNScalar;
  if (from == N  && to == lb) return (1.0f / kgtoNScalar) * kgtolbScalar;
  // Torque conversions
  if (from == Nm && to == lbft) return NmtolbftScalar;
  if (from == lbft && to == Nm) return 1.0f / NmtolbftScalar;
  // Cross-type (e.g. mass → torque) — no conversion
  return 1.0f;
}

float removeNegativeSignFromZero(float weightValue)
{
  float precision = (float) WVAL_PREC / ((float)WVAL_DEC_PLS * 10);
  if((weightValue < precision) && (weightValue > -precision)) weightValue = 0;
  return weightValue;
}

//  https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function
float analogReadVoltage(byte pin){
  float reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if(reading < 1 || reading > 4095) return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  // return -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089 - 0.09;
  return(-0.000000000023926 * pow(reading,3) + 0.000000094746 * pow(reading,2) + 0.00074539 * reading);
} // Added an improved polynomial, use either, comment out as required
