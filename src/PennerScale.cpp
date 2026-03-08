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
#define WVAL_Y_POS 40
#define WVAL_DEC_PLS 1
#define WVAL_PREC 1
#define UNIT_Y_POS 56  // Unit on its own line below weight, right-justified
#define DEC_PT_W_PX 5  // Actual width of the decimal point in px
#define DEC_PT_S_PX 7  // Space generated between numbers to allow for decimal point in px (usually ~1.5x dec pt width)
#define WVAL_FONT u8g2_font_inb21_mn
#define UNIT_FONT u8g2_font_7x13B_mr
#define MSG_FONT u8g2_font_6x12_m_symbols
#define FW_FONT u8g2_font_4x6_mr
#define LCD_CONTRAST 60

// LCD battery indicator formatting constants
#define BAT_IND_WIDTH 14
#define BAT_IND_HEIGHT 6
#define BAT_BUMP_HEIGHT 2
#define BAT_BUMP_WIDTH 2
#define BAT_IND_Y_POS 1  // Top-right corner of display
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

// Global variables
RunningAverage extADCRunAV(EXT_ADC_AVG_SAMPLE_NUM);
int32_t extADCResultCh0 = 0;
int32_t extADCResultCh1 = 0;
int32_t extADCResult = 0;
float extADCweight = 0.0;
float extADCweightMax = 0.0;
float vinVolts = 0.0;
float v5vVolts = 0.0;

bool configSwitch1 = 0;
bool configSwitch2 = 0;

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

e_unitVal unitVal = DEFAULT_UNIT;

float calValue = 7168.220215f; // default global calibration value (429.4967296f results in uV at 5Vexc and 128x gain, 7168.220215f gets close to the dual load cell setup)
int32_t zeroValue = 0; // default zero value
float tareValue = 0.0f; // default tare value
uint32_t calWeight = MIN_CAL_VAL; // default calibration weight
e_unitVal calUnit = DEFAULT_UNIT; // default calibration unit

extern const float kgtolbScalar = 2.20462;
extern const float kgtoNScalar  = 9.80665;
extern const float NmtolbftScalar = 0.737562;
const char* const unitAbbr[] = {"kg", "lb", "N", "Nm", "lbft"};
float stabThreshold   = STAB_THRESH_DEFAULT;
float overloadCapacity = OVER_CAP_DEFAULT;
bool adaptiveFilterEnable = ADAPT_FILTER_DEFAULT;
float adaptiveFilterPct = ADAPT_THRESH_DEFAULT;
uint32_t adaptiveFilterTimeUs = ADAPT_TIME_DEFAULT;
bool adcInvert = false;
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
    calValue = EEPROMcalValue;
    zeroValue = EEPROMZeroValue;
  } else { eepromDirty = true; }
  if (EEPROMbacklightEnable >= 0 && EEPROMbacklightEnable <= 2) {
    backlightEnable = EEPROMbacklightEnable;
  } else { eepromDirty = true; }
  if (EEPROMunitVal >= 0 && EEPROMunitVal < UNIT_VAL_COUNT) {
    unitVal = EEPROMunitVal;
  } else { eepromDirty = true; }
  if (EEPROMcalWeight != 4294967295) {
    calWeight = EEPROMcalWeight;
  } else { eepromDirty = true; }
  if (EEPROMcalUnit >= 0 && EEPROMcalUnit < UNIT_VAL_COUNT)
  {
    calUnit = EEPROMcalUnit;
  }
  else
  {
    calUnit = unitVal;
    eepromDirty = true;
  }
  if (!isnan(EEPROMextADCweightMax)) {
    extADCweightMax = EEPROMextADCweightMax;
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
    stabThreshold = EEPROMstabThresh;
  } else { eepromDirty = true; }

  float EEPROMoverCap;
  EEPROM.get(EEPROM_ADDR_OVER_CAP, EEPROMoverCap);
  if (!isnan(EEPROMoverCap) && EEPROMoverCap > 0) {
    overloadCapacity = EEPROMoverCap;
  } else { eepromDirty = true; }

  uint8_t EEPROMadaptEnable;
  EEPROM.get(EEPROM_ADDR_ADAPT_ENABLE, EEPROMadaptEnable);
  if (EEPROMadaptEnable <= 1) {
    adaptiveFilterEnable = (bool) EEPROMadaptEnable;
  } else { eepromDirty = true; }

  float EEPROMadaptThresh;
  EEPROM.get(EEPROM_ADDR_ADAPT_THRESH, EEPROMadaptThresh);
  if (!isnan(EEPROMadaptThresh) && EEPROMadaptThresh > 0) {
    adaptiveFilterPct = EEPROMadaptThresh;
  } else { eepromDirty = true; }

  uint32_t EEPROMadaptTime;
  EEPROM.get(EEPROM_ADDR_ADAPT_TIME, EEPROMadaptTime);
  if (EEPROMadaptTime >= 1 && EEPROMadaptTime <= 60000000UL) {
    adaptiveFilterTimeUs = EEPROMadaptTime;
  } else { eepromDirty = true; }

  uint8_t EEPROMadcInvert;
  EEPROM.get(EEPROM_ADDR_ADC_INVERT, EEPROMadcInvert);
  if (EEPROMadcInvert <= 1) {
    adcInvert = (bool) EEPROMadcInvert;
  } else { eepromDirty = true; }

  // Write validated defaults back to EEPROM for any uninitialized fields
  if (eepromDirty) {
    EEPROM.put(EEPROM_ADDR_CAL_VALUE, calValue);
    EEPROM.put(EEPROM_ADDR_ZERO_VALUE, zeroValue);
    EEPROM.put(EEPROM_ADDR_BACKLIGHT, backlightEnable);
    EEPROM.put(EEPROM_ADDR_UNIT_VAL, unitVal);
    EEPROM.put(EEPROM_ADDR_CAL_WEIGHT, calWeight);
    EEPROM.put(EEPROM_ADDR_CAL_UNIT, calUnit);
    EEPROM.put(EEPROM_ADDR_WEIGHT_MAX, extADCweightMax);
    EEPROM.put(EEPROM_ADDR_BACKLIGHT_PWM, backlightPWM);
    EEPROM.put(EEPROM_ADDR_ECHO, (uint8_t) scpiEchoEnable);
    EEPROM.put(EEPROM_ADDR_PROMPT, (uint8_t) scpiPromptEnable);
    EEPROM.put(EEPROM_ADDR_STAB_THRESH, stabThreshold);
    EEPROM.put(EEPROM_ADDR_OVER_CAP, overloadCapacity);
    EEPROM.put(EEPROM_ADDR_ADAPT_ENABLE, (uint8_t) adaptiveFilterEnable);
    EEPROM.put(EEPROM_ADDR_ADAPT_THRESH, adaptiveFilterPct);
    EEPROM.put(EEPROM_ADDR_ADAPT_TIME, adaptiveFilterTimeUs);
    EEPROM.put(EEPROM_ADDR_ADC_INVERT, (uint8_t) adcInvert);
    EEPROM.commit();
    DBG_PRINTLN("EEPROM: initialized unset fields with defaults");
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

  DBG_PRINTF("Default calVal: %f\n\r", calValue);
  DBG_PRINTF("Default zeroVal: %d\n\r", zeroValue);
  DBG_PRINTF("Default backlightEnable: %d\n\r", backlightEnable);
  DBG_PRINTF("Default backlightPWM: %d%%\n\r", backlightPWM);
  DBG_PRINTF("Default unitVal: %d\n\r", unitVal);
  DBG_PRINTF("Default calWeight: %u\n\r", calWeight);
  DBG_PRINTF("Default calUnit: %d\n\r", calUnit);
  
  DBG_PRINTF("Max EEPROM address: %d\n\r", EEPROM_ADDR_WEIGHT_MAX);
  DBG_PRINTF("EEPROM calVal: %f\n\r", EEPROMcalValue);
  DBG_PRINTF("EEPROM zeroVal: %d\n\r", EEPROMZeroValue);  
  DBG_PRINTF("EEPROM backlightEnable: %d\n\r", EEPROMbacklightEnable);
  DBG_PRINTF("EEPROM unitVal: %d\n\r", EEPROMunitVal);
  DBG_PRINTF("EEPROM calWeight: %u\n\r", EEPROMcalWeight);
  DBG_PRINTF("EEPROM calUnit: %d\n\r", EEPROMcalUnit);
  DBG_PRINTF("EEPROM extADCweightMax: %f\n\r", EEPROMextADCweightMax);  

  DBG_PRINTF("calVal: %f\n\r", calValue);
  DBG_PRINTF("zeroVal: %u\n\r", zeroValue);
  DBG_PRINTF("backlightEnable: %d\n\r", backlightEnable);
  DBG_PRINTF("unitVal: %d (%s)\n\r", unitVal, unitAbbr[unitVal]);
  DBG_PRINTF("calWeight: %u\n\r", calWeight);
  DBG_PRINTF("calUnit: %d (%s)\n\r", calUnit, unitAbbr[calUnit]);

  // Initialize external ADC

  extADCRunAV.clear();  //   explicitly start our running average buffer clean

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
    extADCResultCh0 = AD7193.singleConversion();

    DBG_PRINTF("ADC Ch0: %d\n", extADCResultCh0);
    AD7193.channelSelect(AD7193_CH_1);
    delay(EXT_ANALOG_READ_TASK_DELAY);
    extADCResultCh1 = AD7193.singleConversion();
    DBG_PRINTF("ADC Ch1: %d\n", extADCResultCh1);

    AD7193.channelSelect(AD7193_CH_0);  // Set the ADC back to Ch0 to get ready for the ADC read task

    // If DIP switch 1 is off (logic high), then scale is configured for single channel.  Otherwise use both channels.
    if (adcInvert) { extADCResultCh0 = -extADCResultCh0; extADCResultCh1 = -extADCResultCh1; }
    if(configSwitch1) extADCResult = extADCResultCh0; else extADCResult = extADCResultCh0 + extADCResultCh1;

    tareValue = (extADCResult - zeroValue)/calValue;
    DBG_PRINTF("Tare Value: %f\n", tareValue);
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
      // Do zero routine
      DBG_PRINTF("Zero scale flag set.\n");
      powerButtonFlag = no_press_flag;
      tareValue = (extADCResult - zeroValue)/calValue;
      extADCRunAV.clear();
      DBG_PRINTF("Tare Value: %f\n", tareValue);
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
      // Change units — cycle through all units, convert capacity/peak within same type
      DBG_PRINTF("Change unit flag set.\n");
      unitButtonFlag = no_press_flag;
      e_unitVal oldUnit = unitVal;
      unitVal = (e_unitVal)((unitVal + 1) % UNIT_VAL_COUNT);
      // Convert overloadCapacity and peak weight between compatible units
      float convFactor = unitConversionFactor(oldUnit, unitVal);
      overloadCapacity *= convFactor;
      extADCweightMax *= convFactor;
      extADCRunAV.clear();
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
  static float lastExtADCweight = 0;
  static float weightChangeLB = 0;
  static int32_t lastExtADCResultRaw = 0;
  static uint32_t adaptiveStartUs = 0;   // Adaptive filter: timestamp of first deviation
  static bool adaptiveLastAbove = false; // Adaptive filter: last deviation direction
  static bool adaptiveTracking = false;  // Adaptive filter: currently tracking a deviation
  msAtLastIdleActivity = millis();  // Initialize idle timer

  xTaskDelayUntil(&xLastWakeTime, EXT_ANALOG_READ_TASK_DELAY/portTICK_PERIOD_MS);

  for (;;){ // A Task shall never return or exit.
    lastExtADCweight = extADCweight;

    // --- Ch0 read (mutex held only during SPI transaction) ---
    // AD7193 library manages its own beginTransaction/endTransaction internally
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

    // Apply polarity inversion to locals before writing globals
    // (prevents SCPI queries from seeing un-negated values during the delay)
    if (adcInvert) { ch0 = -ch0; ch1 = -ch1; }
    extADCResultCh0 = ch0;
    extADCResultCh1 = ch1;
    if(configSwitch1) extADCResult = ch0; else extADCResult = ch0 + ch1;
    extADCweight = (calValue != 0.0f) ? (extADCResult - zeroValue)/calValue - tareValue : 0.0f;
    extADCweight *= unitConversionFactor(calUnit, unitVal);
    extADCweight = removeNegativeSignFromZero(extADCweight);
    // DBG_PRINTF("extADCweight: %f\n", extADCweight);

    // Update shared measurement data (synchronized with SCPI reads)
    taskENTER_CRITICAL(&measMux);
    if (fabsf(extADCweight) > fabsf(extADCweightMax))
    {
      extADCweightMax = extADCweight;
    }
    // Adaptive filter: clear average on sustained directional change
    if (adaptiveFilterEnable && extADCRunAV.getCount() > 0) {
      float avg = extADCRunAV.getAverage();
      float thresh = overloadCapacity * adaptiveFilterPct / 100.0f;
      float delta = extADCweight - avg;
      uint32_t nowUs = micros();
      if (thresh > 0.0f && fabsf(delta) > thresh) {
        bool above = (delta > 0.0f);
        if (!adaptiveTracking || above != adaptiveLastAbove) {
          adaptiveStartUs = nowUs;
          adaptiveLastAbove = above;
          adaptiveTracking = true;
        } else if ((nowUs - adaptiveStartUs) >= adaptiveFilterTimeUs) {
          extADCRunAV.clear();
          adaptiveTracking = false;
        }
      } else {
        adaptiveTracking = false;
      }
    } else {
      adaptiveTracking = false;
    }
    extADCRunAV.add(extADCweight);
    taskEXIT_CRITICAL(&measMux);

    // Signal UI task that new weight data is available
    newWeightReady = true;

    // Compute weight and ADC changes
    int32_t adcChange = abs(extADCResult - lastExtADCResultRaw);
    lastExtADCResultRaw = extADCResult;
    weightChangeLB = abs(lastExtADCweight - extADCweight);
    // Convert weight change to lb for power-down/idle threshold comparison
    weightChangeLB *= unitConversionFactor(unitVal, lb);

    // --- Power-down activity detection (original thresholds) ---
    bool powerDownActivity = (calValue != 0.0f)
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
    float fullScaleWeight = (float)ADC_FULL_SCALE / calValue;  // in calUnit
    float idleThresholdLB = fullScaleWeight * unitConversionFactor(calUnit, lb) * IDLE_ACTIVITY_PCT / 100.0f;
    int32_t idleThresholdADC = (int32_t)((float)ADC_FULL_SCALE * IDLE_ACTIVITY_PCT / 100.0f);
    bool idleActivity = (calValue != 0.0f)
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

void UpdateWeightReadingLCD()
{
  if(updateLCDWeight)
  {
    String s_extADCweight;

    //char vBuffer[7];
    //dtostrf(extADCweight,5,2, vBuffer);  // Cannot use printf with floats with small task sizes (<2048) - so do this instead
    //DBG_PRINTF("ExtADC: %s\n", vBuffer);

    u8g2.clearBuffer();
    u8g2.setFont(WVAL_FONT);
    s_extADCweight = String(extADCRunAV.getAverage(), WVAL_DEC_PLS);
    // Split string at the decimal point because display looks strange with large decimal point spacing

    // Print decimal point first
    u8g2.setCursor(WVAL_X_POS-u8g2.getStrWidth(s_extADCweight.substring(s_extADCweight.length()-WVAL_DEC_PLS).c_str())-DEC_PT_S_PX+((DEC_PT_S_PX-DEC_PT_W_PX)/2)-DEC_PT_W_PX-1, WVAL_Y_POS);
    u8g2.print(".");

    // Print numbers left of the deicmal place
    u8g2.setCursor(WVAL_X_POS-u8g2.getStrWidth((s_extADCweight.c_str()+1))-DEC_PT_S_PX, WVAL_Y_POS);
    u8g2.print(s_extADCweight.substring(0, s_extADCweight.length()-WVAL_DEC_PLS-1).c_str());  
    
    // Print numbers right of the decimal place
    u8g2.setCursor(WVAL_X_POS-u8g2.getStrWidth(s_extADCweight.substring(s_extADCweight.length()-WVAL_DEC_PLS).c_str()), WVAL_Y_POS);
    u8g2.print(s_extADCweight.substring(s_extADCweight.length()-WVAL_DEC_PLS).c_str());

    u8g2.setFont(UNIT_FONT);
    const char *uStr = unitAbbr[unitVal];
    u8g2.setCursor(u8g2.getDisplayWidth() - u8g2.getStrWidth(uStr), UNIT_Y_POS);
    u8g2.print(uStr);

    // Print battery indicator
    u8g2.drawFrame(BAT_IND_X_POS, BAT_IND_Y_POS, BAT_IND_WIDTH - BAT_BUMP_WIDTH, BAT_IND_HEIGHT);
    u8g2.drawFrame(BAT_IND_X_POS + BAT_IND_WIDTH - BAT_BUMP_WIDTH, BAT_IND_Y_POS + BAT_IND_HEIGHT/2 - BAT_BUMP_HEIGHT/2, BAT_IND_X_POS - BAT_BUMP_WIDTH, BAT_BUMP_WIDTH);  
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

  // Set calibration units
  calUnit = unitVal;
  u8g2.clearBuffer();
  u8g2.setCursor(0, USER_MSG_Y_POS);
  u8g2.print("Press UNIT to select"); 
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
  u8g2.printf("calibration units: %s", unitAbbr[calUnit]);
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT * 2);
  u8g2.print("Press ZERO to cont.");
  sendBufferSPISafe();

  // Allow user to release ZERO before proceeding
  while (powerButtonFlag != no_press_flag)
  {
    vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS);
  }

  // Wait for user to press ZERO
  while (powerButtonFlag == no_press_flag)
  {
    // Check unit button flag
    if (unitButtonFlag == short_press_flag)
    {
      // Change cal units
      DBG_PRINTF("Change cal unit flag set.\n");
      unitButtonFlag = no_press_flag;
      calUnit = (e_unitVal)((calUnit + 1) % UNIT_VAL_COUNT);
      u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
      u8g2.printf("calibration units: %-4s", unitAbbr[calUnit]);
      sendBufferSPISafe();
    }

    vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS);
  }

  // Reset button flag
  powerButtonFlag = no_press_flag;

  // Set calibration weight
  u8g2.clearBuffer();
  u8g2.setCursor(0, USER_MSG_Y_POS);
  u8g2.print("Press ");
  u8g2.drawGlyph(u8g2.getCursorX(), u8g2.getCursorY(), 0x25e0); // Up arrow
  u8g2.setCursor (u8g2.getCursorX() + u8g2.getMaxCharWidth(), u8g2.getCursorY());
  u8g2.print(" or ");
  u8g2.drawGlyph(u8g2.getCursorX(), u8g2.getCursorY(), 0x25e1);   // Down arrow
  u8g2.setCursor (u8g2.getCursorX() + u8g2.getMaxCharWidth(), u8g2.getCursorY());
  u8g2.print(" to set");
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);  
  u8g2.printf("cal. weight: %5u %s", calWeight, unitAbbr[calUnit]);
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT * 2);
  u8g2.print("Press ZERO to cont.");
  sendBufferSPISafe();

// Wait for user to press ZERO
  while (powerButtonFlag == no_press_flag)
  {
    if (bklButtonStat)
    {
      if (bklButtonStat == long_press)
      {
        // User is holding down the button
        if (calWeight < (MAX_CAL_VAL-10)) calWeight += 10;
        else calWeight = MAX_CAL_VAL;
      }
      else
      {
        if (calWeight < (MAX_CAL_VAL)) calWeight ++;
      } 
      u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);  
      u8g2.printf("cal. weight: %5u %s", calWeight, unitAbbr[calUnit]); 
      sendBufferSPISafe();
    } 
    if (unitButtonStat)
    {
      if (unitButtonStat == long_press)
      {
        // User is holding down the button
        if (calWeight > (MIN_CAL_VAL + 10)) calWeight -= 10;
        else calWeight = MIN_CAL_VAL;
      }
      else
      {
        if (calWeight > MIN_CAL_VAL) calWeight --;
      } 
      u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);  
      u8g2.printf("cal. weight: %5u %s", calWeight, unitAbbr[calUnit]); 
      sendBufferSPISafe();
    }
    vTaskDelay(150/portTICK_PERIOD_MS);
  }

  // Commit calWeight and calUnit to EEPROM
  EEPROM.put(EEPROM_ADDR_CAL_WEIGHT, calWeight);
  EEPROM.put(EEPROM_ADDR_CAL_UNIT, calUnit);  
  EEPROM.commit();

  // Reset button flags
  powerButtonFlag = no_press_flag;
  unitButtonFlag = no_press_flag;
  bklButtonFlag = no_press_flag;

  u8g2.clearBuffer();
  u8g2.setCursor(0, USER_MSG_Y_POS);
  u8g2.print("--Scale Calibration--");
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);  
  u8g2.print("Unload scale then");
  u8g2.setCursor(0, USER_MSG_Y_POS + 2 * USER_MSG_Y_LINE_HEIGHT);
  u8g2.print("press ZERO.");
  sendBufferSPISafe();
  while (powerButtonFlag == no_press_flag)
  {
    vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS);
  }

  // Reset button flags
  powerButtonFlag = no_press_flag;
  unitButtonFlag = no_press_flag;

  extADCRunAV.clear();
  u8g2.clearBuffer();
  u8g2.setCursor(0, USER_MSG_Y_POS);
  u8g2.print("--Scale Calibration--");  
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
  u8g2.printf("Wait.");
  sendBufferSPISafe();
  vTaskDelay(1000/portTICK_PERIOD_MS);
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
  u8g2.printf("Wait..");
  sendBufferSPISafe();
  vTaskDelay(1000/portTICK_PERIOD_MS);
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
  u8g2.printf("Wait...");
  sendBufferSPISafe();
  vTaskDelay(1000/portTICK_PERIOD_MS);
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
  u8g2.printf("Wait....");  
  sendBufferSPISafe();
  vTaskDelay(1000/portTICK_PERIOD_MS);
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
  u8g2.printf("Wait.....");   
  sendBufferSPISafe();
  vTaskDelay(1000/portTICK_PERIOD_MS);

  // Store zero
  zeroValue = extADCResult;

  u8g2.clearBuffer();
  u8g2.setCursor(0, USER_MSG_Y_POS);
  u8g2.print("--Scale Calibration--");
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);  
  u8g2.printf("Load %d %s then press", calWeight, unitAbbr[calUnit]);
  u8g2.setCursor(0, USER_MSG_Y_POS + 2 * USER_MSG_Y_LINE_HEIGHT);
  u8g2.print("ZERO.");
  sendBufferSPISafe();
  while (powerButtonFlag == no_press_flag)
  {
    vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS);
  }

  // Reset button flags
  powerButtonFlag = no_press_flag;
  unitButtonFlag = no_press_flag;
  
  extADCRunAV.clear();
  u8g2.clearBuffer();
  u8g2.setCursor(0, USER_MSG_Y_POS);
  u8g2.print("--Scale Calibration--");  
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
  u8g2.printf("Wait.");
  sendBufferSPISafe();
  vTaskDelay(1000/portTICK_PERIOD_MS);
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
  u8g2.printf("Wait..");
  sendBufferSPISafe();
  vTaskDelay(1000/portTICK_PERIOD_MS);
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
  u8g2.printf("Wait...");
  sendBufferSPISafe();
  vTaskDelay(1000/portTICK_PERIOD_MS);
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
  u8g2.printf("Wait....");  
  sendBufferSPISafe();
  vTaskDelay(1000/portTICK_PERIOD_MS);
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);
  u8g2.printf("Wait....."); 
  sendBufferSPISafe();
  vTaskDelay(1000/portTICK_PERIOD_MS);

  // Store span
  calValue = ((float)extADCResult - (float)zeroValue)/(float)calWeight;  

  DBG_PRINTF("extADCResult: %d\n", extADCResult);
  DBG_PRINTF("zeroValue: %d\n", zeroValue);
  DBG_PRINTF("calWeight: %u\n", calWeight);
  DBG_PRINTF("calValue: %f\n", calValue);
  DBG_PRINTF("calUnit: %s\n", unitAbbr[calUnit]);  

  u8g2.clearBuffer();
  u8g2.setCursor(0, USER_MSG_Y_POS);
  u8g2.print("Cal done. Press ZERO.");  
  u8g2.setCursor(0, USER_MSG_Y_POS + USER_MSG_Y_LINE_HEIGHT);  
  u8g2.printf("Zero: %d\n\r", zeroValue);
  u8g2.setCursor(0, USER_MSG_Y_POS + 2 * USER_MSG_Y_LINE_HEIGHT);
  u8g2.printf("Span: %f\n\r", calValue);  
  sendBufferSPISafe();
  while (powerButtonFlag == no_press_flag)
  {
    vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS);
  }  

  extADCRunAV.clear();
  // Newly calibrated scale - do not need tare
  tareValue = 0;
  EEPROM.put(EEPROM_ADDR_CAL_VALUE, calValue);
  EEPROM.put(EEPROM_ADDR_CAL_UNIT, calUnit);
  EEPROM.put(EEPROM_ADDR_ZERO_VALUE, zeroValue);
  EEPROM.commit();

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
  e_backlightEnable EEPROMbacklightEnable;
  e_unitVal EEPROMunitVal;
  float EEPROMextADCweightMax;
  uint8_t EEPROMbacklightPWM;

  // Check EEPROM values and update if necessary
  EEPROM.get(EEPROM_ADDR_BACKLIGHT, EEPROMbacklightEnable);
  EEPROM.get(EEPROM_ADDR_UNIT_VAL, EEPROMunitVal);
  EEPROM.get(EEPROM_ADDR_WEIGHT_MAX, EEPROMextADCweightMax);
  EEPROM.get(EEPROM_ADDR_BACKLIGHT_PWM, EEPROMbacklightPWM);

  if(EEPROMunitVal != unitVal) EEPROM.put(EEPROM_ADDR_UNIT_VAL, unitVal);
  if(EEPROMbacklightEnable != backlightEnable) EEPROM.put(EEPROM_ADDR_BACKLIGHT, backlightEnable);
  if(EEPROMbacklightPWM != backlightPWM) EEPROM.put(EEPROM_ADDR_BACKLIGHT_PWM, backlightPWM);

  uint8_t EEPROMecho;
  EEPROM.get(EEPROM_ADDR_ECHO, EEPROMecho);
  if (EEPROMecho != (uint8_t)scpiEchoEnable) EEPROM.put(EEPROM_ADDR_ECHO, (uint8_t)scpiEchoEnable);

  uint8_t EEPROMprompt;
  EEPROM.get(EEPROM_ADDR_PROMPT, EEPROMprompt);
  if (EEPROMprompt != (uint8_t)scpiPromptEnable) EEPROM.put(EEPROM_ADDR_PROMPT, (uint8_t)scpiPromptEnable);

  float EEPROMstabThresh;
  EEPROM.get(EEPROM_ADDR_STAB_THRESH, EEPROMstabThresh);
  if (EEPROMstabThresh != stabThreshold) EEPROM.put(EEPROM_ADDR_STAB_THRESH, stabThreshold);

  float EEPROMoverCap;
  EEPROM.get(EEPROM_ADDR_OVER_CAP, EEPROMoverCap);
  if (EEPROMoverCap != overloadCapacity) EEPROM.put(EEPROM_ADDR_OVER_CAP, overloadCapacity);

  uint8_t EEPROMadaptEn;
  EEPROM.get(EEPROM_ADDR_ADAPT_ENABLE, EEPROMadaptEn);
  if (EEPROMadaptEn != (uint8_t)adaptiveFilterEnable) EEPROM.put(EEPROM_ADDR_ADAPT_ENABLE, (uint8_t)adaptiveFilterEnable);

  float EEPROMadaptThr;
  EEPROM.get(EEPROM_ADDR_ADAPT_THRESH, EEPROMadaptThr);
  if (EEPROMadaptThr != adaptiveFilterPct) EEPROM.put(EEPROM_ADDR_ADAPT_THRESH, adaptiveFilterPct);

  uint32_t EEPROMadaptTime;
  EEPROM.get(EEPROM_ADDR_ADAPT_TIME, EEPROMadaptTime);
  if (EEPROMadaptTime != adaptiveFilterTimeUs) EEPROM.put(EEPROM_ADDR_ADAPT_TIME, adaptiveFilterTimeUs);

  DBG_PRINTF("EEPROMextADCweightMax: %f\n", EEPROMextADCweightMax);
  DBG_PRINTF("extADCweightMax: %f\n", extADCweightMax);
  if(extADCweightMax != EEPROMextADCweightMax)
  {
    EEPROM.put(EEPROM_ADDR_WEIGHT_MAX, extADCweightMax);
    DBG_PRINTF("Updating extADCweightMax: %f", extADCweightMax);
  }
  vTaskDelay(50/portTICK_PERIOD_MS);
  EEPROM.commit();  // Save our settings to EEPROM
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
