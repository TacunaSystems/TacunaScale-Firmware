/* Penner Bathing Scale */

// Includes
#include <Arduino.h>
#include <U8g2lib.h>
#include <PRDC_AD7193.h>
#include <EEPROM.h>

// Library defines
// FreeRTOS
#define ARDUINO_RUNNING_CORE 1
// U8g2lib
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// Pin defines
// LCD
#define LCD_RST 15
#define LCD_A0 16
#define LCD_CS 17
#define LCD_BACKLIGHT 14

// User
#define PWR_ZERO_BTN 19
#define UNIT_BTN 20
#define AUX_BTN 21

// Power
#define V3V3_EN 2
#define VIN_LVL_EN 4
#define VIN_LVL 5
#define V5_A_LVL 6
#define V5_A_EN 7
#define V3V3_PG 18
#define V5_A_PG 35

// EXT ADC
#define SCLK 12
#define MISO 13
#define MOSI 11
#define EXT_ADC_CS 10

// Constants
#define VIN_LVL_PIN 5 //  10k & 3.3k Vdiv => 1/0.248
#define VIN_LVL_COUNTS_TO_V 0.0032486518f // Multiply counts by this scalar to get Volts. => 1/(4096/3.3)*0.248
#define PWR_5V_LVL_COUNTS_TO_V 0.001611328125f // Multiply counts by this scalar to get Volts. => 1/(4096/3.3)*0.5
#define VIN_LVL_EN_PIN 4
#define PWR_5V_A_LVL_PIN 6

// UI
#define LONG_PRESS_LOOP_DELAY 50 // 50ms 
#define LONG_PRESS_COUNT_THRESHOLD 40 // multiples of LONG_PRESS_LOOP_DELAY (ex. for LONG_PRESS_DELAY = 50ms (40 counts = 2s)
#define DEBOUNCE_TIME 25
#define SCALE_CAP 500
#define SCALE_CAP_UNIT lb
#define MIN_CAL_VAL 100
#define MAX_CAL_VAL 500
#define DEFAULT_UNIT lb

// LCD weight reading formatting constants
#define WVAL_X_POS 112 // Reading is rigit-aligned, so x position should be the right most location
#define WVAL_Y_POS 37
#define WVAL_DEC_PLS 1
#define WVAL_PREC 1
#define UNIT_X_POS 115
#define UNIT_Y_POS 37
#define DEC_PT_W_PX 5  // Actual width of the decimal point in px
#define DEC_PT_S_PX 9  // Space generated between numbers to allow for decimal point in px (usually ~1.5x dec pt width)
#define WVAL_FONT u8g2_font_inb21_mn
#define UNIT_FONT u8g2_font_7x13B_mr
#define MSG_FONT u8g2_font_6x12_mr

// U8g2 Contructor (Frame Buffer)
U8G2_ST7567_ENH_DG128064I_F_4W_SW_SPI u8g2(U8G2_R2, /* clock=*/ SCLK, /* data=*/ MOSI, /* cs=*/ LCD_CS, /* dc=*/ LCD_A0, /* reset=*/ LCD_RST); 

// AD7193 Constructor
PRDC_AD7193 AD7193;

//Penner Logo XBM
#define penner_logo_width 114
#define penner_logo_height 28
static uint8_t penner_logo_bits[] = {
  0x00, 0xC0, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xF0, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xF0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0xC0, 
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x1C, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x0E, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x03, 0x00, 0xF0, 
  0x07, 0xFF, 0x18, 0x18, 0x03, 0xE3, 0x1F, 0x7F, 0x00, 0x00, 0x0E, 0x00, 
  0x03, 0x00, 0xF0, 0x0F, 0xFF, 0x18, 0x18, 0x03, 0xE3, 0x1F, 0xFF, 0x00, 
  0xC0, 0x07, 0x01, 0x03, 0x00, 0x30, 0x1C, 0x03, 0x38, 0x18, 0x07, 0x63, 
  0x00, 0xC3, 0x01, 0xCF, 0x87, 0x01, 0x83, 0x78, 0x30, 0x18, 0x03, 0x78, 
  0x18, 0x0F, 0x63, 0x00, 0x83, 0x01, 0xCF, 0x87, 0x81, 0x83, 0x78, 0x30, 
  0x1C, 0x03, 0xF8, 0x18, 0x1F, 0x63, 0x00, 0xC3, 0x01, 0xCF, 0x8F, 0xC3, 
  0xC1, 0x78, 0xF0, 0x0F, 0x7F, 0xD8, 0x19, 0x3B, 0xE3, 0x0F, 0xFF, 0x00, 
  0x9F, 0x0F, 0xFF, 0xC0, 0x7C, 0xF0, 0x07, 0x7F, 0x98, 0x1B, 0x73, 0xE3, 
  0x0F, 0x3F, 0x00, 0x9F, 0x0F, 0x7E, 0x60, 0x7C, 0x30, 0x00, 0x03, 0x18, 
  0x1F, 0xE3, 0x63, 0x00, 0x7F, 0x00, 0x1F, 0x1F, 0x1C, 0x70, 0x7C, 0x30, 
  0x00, 0x03, 0x18, 0x1E, 0xC3, 0x63, 0x00, 0xE3, 0x00, 0x3F, 0x1F, 0x00, 
  0x38, 0x7E, 0x30, 0x00, 0x03, 0x18, 0x1C, 0x83, 0x63, 0x00, 0xC3, 0x01, 
  0x3F, 0x3E, 0x00, 0x3C, 0x7E, 0x30, 0x00, 0xFF, 0x18, 0x18, 0x03, 0xE3, 
  0x1F, 0x83, 0x03, 0x7F, 0x7C, 0x00, 0x1E, 0x7F, 0x30, 0x00, 0xFF, 0x18, 
  0x18, 0x03, 0xE3, 0x1F, 0x03, 0x03, 0x7F, 0xF8, 0xC1, 0x0F, 0x7F, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xF0, 0xFF, 
  0x87, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xFF, 0xE1, 0xFF, 0xC3, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xFF, 0x83, 0xFF, 0xE1, 0x7F, 0xF0, 0x41, 0xF8, 0x44, 
  0x12, 0xF1, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x0F, 0x1C, 0xF0, 0x7F, 0x10, 
  0xE2, 0x20, 0x44, 0x12, 0x09, 0x01, 0x00, 0x00, 0x00, 0xFF, 0x1F, 0x00, 
  0xFC, 0x7F, 0x10, 0xA3, 0x20, 0x44, 0x32, 0x09, 0x00, 0x00, 0x00, 0x00, 
  0xFF, 0xFF, 0xC1, 0xFF, 0x7F, 0xF0, 0x11, 0x21, 0x7C, 0x52, 0xC9, 0x01, 
  0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0x3F, 0x10, 0xF2, 0x21, 0x44, 
  0x92, 0x09, 0x01, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0x1F, 0x10, 
  0x12, 0x21, 0x44, 0x12, 0x09, 0x01, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 
  0xFF, 0x0F, 0xF0, 0x11, 0x21, 0x44, 0x12, 0xF1, 0x00, 0x00, 0x00, 0x00, 
  };

// Define tasks
void TaskUpdateWeightReadingLCD( void *pvParameters );
void TaskIntAnalogRead( void *pvParameters );
void TaskPowerZeroButton( void *pvParameters );
void TaskUnitButton( void *pvParameters );
void TaskUI( void *pvParameters );

SemaphoreHandle_t SPImutex = xSemaphoreCreateMutex();

// Global variables
int32_t extADCresult = 0;
float extADCweight = 0.0;

enum e_backlightEnable {off = 0, on = 1, on_motion = 2};
e_backlightEnable backlightEnable = off;
enum e_unitVal {kg = 0, lb = 1};
e_unitVal unitVal = DEFAULT_UNIT;

float calValue = 1.0; // default global calibration value
int32_t zeroValue = 0; // default zero value
float tareValue = 0.0; // default tare value
uint32_t calWeight = MIN_CAL_VAL; // default calibration weight
e_unitVal calUnit = DEFAULT_UNIT; // default calibration unit

const float kgtolbScalar = 2.20462;
const String unitAbbr[] = {"kg", "lb"};
bool updateLCDWeight = true;

const int calVal_eepromAdress = 0;
const int zeroVal_eepromAdress = calVal_eepromAdress + sizeof(calValue);
const int backlightEnable_eepromAdress = zeroVal_eepromAdress + sizeof(zeroValue);
const int unitVal_eepromAdress = backlightEnable_eepromAdress + sizeof(backlightEnable);
const int calWeight_eepromAdress = unitVal_eepromAdress + sizeof(unitVal);
const int calUnit_eepromAdress = calWeight_eepromAdress + sizeof(calWeight);

enum e_buttonStat {no_press = 0, is_pressed = 1, short_press = 2, long_press = 3};
e_buttonStat powerButtonStat = no_press;  // Used to track immediate button status
e_buttonStat unitButtonStat = no_press;  // Used to track immediate button status
e_buttonStat auxButtonStat = no_press;  // Used to track immediate button status

enum e_buttonFlag {no_press_flag = 0, short_press_flag = 1, long_press_flag = 2};
e_buttonFlag powerButtonFlag = no_press_flag;  // Used to alert other tasks of a button event - cleared by UI task
e_buttonFlag unitButtonFlag = no_press_flag; // Used to alert other tasks of a button event - cleared by UI task
e_buttonFlag auxButtonFlag = no_press_flag; // Used to alert other tasks of a button event - cleared by UI task

long debounceTime = 200;

TaskHandle_t xHandleTaskExtAnalogRead = NULL;
TaskHandle_t xHandleTaskIntAnalogRead = NULL;
TaskHandle_t xHandleTaskUpdateWeightLCD = NULL;
TaskHandle_t xHandleTaskPowerZeroButton = NULL;
TaskHandle_t xHandleTaskUnitButton = NULL;
TaskHandle_t xHandleTaskAuxButton = NULL;
TaskHandle_t xHandleTaskUI = NULL;

// The setup function runs once when you press reset or power on the board.
void setup() {
  // Load cell init
  float EEPROMcalValue; // calibration value from EEPROM
  uint32_t EEPROMZeroValue; // zero value from EEPROM
  e_backlightEnable EEPROMbacklightEnable;
  e_unitVal EEPROMunitVal;
  uint32_t EEPROMcalWeight;
  e_unitVal EEPROMcalUnit;

  // Enable 3.3V 
  pinMode(V3V3_EN, OUTPUT);
  digitalWrite(V3V3_EN, HIGH);

  // Enable 5V_A
  pinMode(V5_A_EN, OUTPUT);
  digitalWrite(V5_A_EN, HIGH);

  // Disable LCD backlight
  pinMode(LCD_BACKLIGHT, OUTPUT);
  digitalWrite(LCD_BACKLIGHT, LOW);


  // Set external ADC CS inactive
  pinMode(EXT_ADC_CS, OUTPUT);
  digitalWrite(EXT_ADC_CS, HIGH);


  // Set LCD CS inactive
  pinMode(LCD_CS, OUTPUT);
  digitalWrite(LCD_CS, HIGH);


  // User buttons
  pinMode(PWR_ZERO_BTN, INPUT);
  pinMode(UNIT_BTN, INPUT);
  pinMode(AUX_BTN, INPUT);

  // Set SPI directions
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCLK, OUTPUT);
  
  // Set internal ADC input
  pinMode(VIN_LVL_PIN, INPUT);
  pinMode(PWR_5V_A_LVL_PIN, INPUT);
  pinMode(VIN_LVL_EN_PIN, OUTPUT);
  digitalWrite(VIN_LVL_EN_PIN, LOW);


  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  delay(500);

  SPI.begin(SCLK, MISO, MOSI, LCD_CS);
  // Start LCD
  u8g2.begin();
  // Set contrast
  u8g2.setContrast(65);
  // Display logo
  u8g2.clearBuffer();
  drawLogo();
  u8g2.sendBuffer();
  SPI.end();
  digitalWrite(LCD_CS, HIGH);

  Serial.printf("Default calVal: %f\n\r", calValue);
  Serial.printf("Default zeroVal: %d\n\r", zeroValue);
  Serial.printf("Default backlightEnable: %d\n\r", backlightEnable);
  Serial.printf("Default unitVal: %d\n\r", unitVal);
  Serial.printf("Default calWeight: %u\n\r", calWeight);
  Serial.printf("Default calUnit: %d\n\r", calUnit);

  EEPROM.begin(512);
  EEPROM.get(calVal_eepromAdress, EEPROMcalValue);
  EEPROM.get(zeroVal_eepromAdress, EEPROMZeroValue);
  EEPROM.get(backlightEnable_eepromAdress, EEPROMbacklightEnable);
  EEPROM.get(unitVal_eepromAdress, EEPROMunitVal);  
  EEPROM.get(calWeight_eepromAdress, EEPROMcalWeight);
  EEPROM.get(calUnit_eepromAdress, EEPROMcalUnit);
  
  Serial.printf("EEPROM calVal: %f\n\r", EEPROMcalValue);
  Serial.printf("EEPROM zeroVal: %d\n\r", EEPROMZeroValue);  
  // If calValue is a real number, that means we've calibrated the unit and calValue and zeroValue are legitimate values
  if ((!isnan(EEPROMcalValue)) && (EEPROMcalValue>0))
  {
    calValue = EEPROMcalValue;
    zeroValue = EEPROMZeroValue;
  } 
  
  Serial.printf("EEPROM backlightEnable: %d\n\r", EEPROMbacklightEnable);
  if (EEPROMbacklightEnable >= 0 && EEPROMbacklightEnable <= 2) backlightEnable = EEPROMbacklightEnable;
  Serial.printf("EEPROM unitVal: %d\n\r", EEPROMunitVal);
  if (EEPROMunitVal >= 0 && EEPROMunitVal <= 1) unitVal = EEPROMunitVal;  
  Serial.printf("EEPROM calWeight: %u\n\r", EEPROMcalWeight);
  if (EEPROMcalWeight != 4294967295) calWeight = EEPROMcalWeight;  
  Serial.printf("EEPROM calUnit: %d\n\r", EEPROMcalUnit);
  if (EEPROMcalUnit >= 0 && EEPROMcalUnit <= 1)
  {
    calUnit = EEPROMcalUnit; 
  }
  else
  {
    calUnit = unitVal;
  }

  Serial.printf("calVal: %f\n\r", calValue);
  Serial.printf("zeroVal: %u\n\r", zeroValue);
  Serial.printf("backlightEnable: %d\n\r", backlightEnable);
  Serial.printf("unitVal: %d (%s)\n\r", unitVal, unitAbbr[unitVal]);
  Serial.printf("calWeight: %u\n\r", calWeight);
  Serial.printf("calUnit: %d (%s)\n\r", calUnit, unitAbbr[calUnit]);

  digitalWrite(LCD_BACKLIGHT, backlightEnable);  

  // Initialize external ADC
  SPI.begin(SCLK, MISO, MOSI, EXT_ADC_CS);
  AD7193.setSPI(SPI);
  AD7193.begin();
  if(0){ //!AD7193.begin()) {
    Serial.println(F("AD7193 initialization failed!"));
  } else {
    AD7193.printAllRegisters();
    AD7193.setClockMode(AD7193_CLK_INT);
    AD7193.setRate(0x001);
    AD7193.setFilter(AD7193_MODE_SINC4);
    AD7193.enableNotchFilter(false);
    AD7193.enableChop(false);
    AD7193.enableBuffer(true);
    AD7193.rangeSetup(1, AD7193_CONF_GAIN_128);
    AD7193.channelSelect(AD7193_CH_0);
    AD7193.setBPDSW(true);
    Serial.println(F("AD7193 Initialized!"));
    delay(250);
    extADCresult = AD7193.singleConversion();
    tareValue = (extADCresult - zeroValue)/calValue;
    Serial.printf("ADC Result: %d\n", extADCresult);
    Serial.printf("Tare Value: %f\n", tareValue);
    delay(250);
    extADCresult = AD7193.singleConversion();
    tareValue = (extADCresult - zeroValue)/calValue;
    Serial.printf("ADC Result: %d\n", extADCresult);
    Serial.printf("Tare Value: %f\n", tareValue);
    delay(250);
    extADCresult = AD7193.singleConversion();
    tareValue = (extADCresult - zeroValue)/calValue;
    Serial.printf("ADC Result: %d\n", extADCresult);
    Serial.printf("Tare Value: %f\n", tareValue);    
    delay(250);
    extADCresult = AD7193.singleConversion();
    tareValue = (extADCresult - zeroValue)/calValue;
    Serial.printf("ADC Result: %d\n", extADCresult);
    Serial.printf("Tare Value: %f\n", tareValue);    
    SPI.end();
  }
  
  // Splash screen delay
  delay(750);
  u8g2.clearBuffer();
  u8g2.sendBuffer();

  // Allow user to release button before proceeding
  while (digitalRead(PWR_ZERO_BTN))
  {
    delay(100);
  }

  // Setup SPI mutex
  assert(SPImutex);

  // Set up tasks
 
  xTaskCreatePinnedToCore(
    TaskExtAnalogRead
    ,  "Ext ADC Task" // A name just for humans
    ,  2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  NULL // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  4  // Priority
    ,  &xHandleTaskExtAnalogRead
    ,  ARDUINO_RUNNING_CORE
    );

  delay(100); // Separate the two tasks using the SPI bus by a short time to try to avoid writing at the same time (we also have a mutex)

  xTaskCreatePinnedToCore(
    TaskUpdateWeightReadingLCD
    ,  "Update Weight LCD"
    ,  4096  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  3  // Priority
    ,  &xHandleTaskUpdateWeightLCD // With task handle we will be able to manipulate with this task.
    ,  ARDUINO_RUNNING_CORE // Core on which the task will run
    );

  xTaskCreatePinnedToCore(
    TaskIntAnalogRead
    ,  "Analog Read"
    ,  2048  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  2  // Priority
    ,  &xHandleTaskIntAnalogRead
    ,  ARDUINO_RUNNING_CORE // Core on which the task will run
    );

    xTaskCreatePinnedToCore(
    TaskPowerZeroButton
    ,  "Check Power/Zero Button"
    ,  2048  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  3  // Priority
    ,  &xHandleTaskPowerZeroButton // With task handle we will be able to manipulate with this task.
    ,  ARDUINO_RUNNING_CORE // Core on which the task will run
    );

    xTaskCreatePinnedToCore(
    TaskUnitButton
    ,  "Check Unit Button"
    ,  2048  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  3  // Priority
    ,  &xHandleTaskUnitButton // With task handle we will be able to manipulate with this task.
    ,  ARDUINO_RUNNING_CORE // Core on which the task will run
    );

    xTaskCreatePinnedToCore(
    TaskAuxButton
    ,  "Check Aux Button"
    ,  2048  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  3  // Priority
    ,  &xHandleTaskAuxButton // With task handle we will be able to manipulate with this task.
    ,  ARDUINO_RUNNING_CORE // Core on which the task will run
    );    

    xTaskCreatePinnedToCore(
    TaskUI
    ,  "UI Task"
    ,  4096  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  4  // Priority
    ,  &xHandleTaskUI // With task handle we will be able to manipulate with this task.
    ,  ARDUINO_RUNNING_CORE // Core on which the task will run
    );


  Serial.printf("Scale initialized.\n");
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop(){
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskUI(void *pvParameters)
{
  ( void ) pvParameters;
  for (;;)
  {
    // Check power button flag
    if (powerButtonFlag == short_press_flag)
    {
      // Do zero routine
      Serial.printf("Zero scale flag set.\n");
      powerButtonFlag = no_press_flag;
      tareValue = (extADCresult - zeroValue)/calValue;
      Serial.printf("Tare Value: %f\n", tareValue);
    }
    else if (powerButtonFlag == long_press_flag)
    {
      if(unitButtonStat == is_pressed || unitButtonFlag == long_press_flag)
      {
        // We've received a press with power button reaching the long press status first.
        // If both buttons have reached long press status, execute task.
        // Otherwise come back and check if unit button has reached long press status.
        // Note this is 1 of 2 places where this can occur...
        // The other is in the unit button tests below.

        if(unitButtonFlag == long_press_flag)
        {
          // Both buttons have reached long-press status.  Execute double long-press task.
          powerButtonFlag = no_press_flag;
          unitButtonFlag = no_press_flag;
          Serial.printf("Calibrate flag set.\n");
          doCalibration();
        }

      }
      else
      {
        // Do power down
        Serial.printf("Power down flag set.\n");
        powerButtonFlag = no_press_flag;
        powerDown();
      }
    }

    // Check unit button flag
    if (unitButtonFlag == short_press_flag)
    {
      // Change units
      Serial.printf("Change unit flag set.\n");
      unitButtonFlag = no_press_flag;
      if(unitVal == kg) unitVal = lb;
      else if(unitVal == lb) unitVal = kg;
      EEPROM.put(unitVal_eepromAdress, unitVal);  
    }
    else if (unitButtonFlag == long_press_flag)
    {
      if(powerButtonStat == is_pressed || powerButtonFlag == long_press_flag)
      {
        // We've received a press with power button reaching the long press status first.
        // If both buttons have reached long press status, execute task.
        // Otherwise come back and check if unit button has reached long press status.
        // Note this is 1 of 2 places where this can occur...
        // The other is in the power button tests above.

        if(powerButtonFlag == long_press_flag)
        {
          // Both buttons have reached long-press status.  Execute double long-press task.
          powerButtonFlag = no_press_flag;
          unitButtonFlag = no_press_flag;
          Serial.printf("Calibrate flag set.\n");
        }
      }
      else
      {
        // Cycle backlight setting
        if(backlightEnable == off) backlightEnable = on;
        else if(backlightEnable == on) backlightEnable = off;
        //else if(backlightEnable == on) backlightEnable = on_motion;
        //else if(backlightEnable == on_motion) backlightEnable = off;        
        EEPROM.put(backlightEnable_eepromAdress, backlightEnable);
        Serial.printf("Backlight toggle flag set. Backlight = %d\n", backlightEnable);
        digitalWrite(LCD_BACKLIGHT, (backlightEnable != off));
        unitButtonFlag = no_press_flag;
      }
    }

    // Check aux button flag
    if (auxButtonFlag == short_press_flag)
    {
      // Do aux button short press action
      auxButtonFlag = no_press_flag;

    }
    else if(auxButtonFlag == long_press_flag)
    {
      // Do aux button long press action
      auxButtonFlag = no_press_flag;
    }
    vTaskDelay(25/portTICK_PERIOD_MS);
  }
}

void TaskExtAnalogRead(void *pvParameters)
{
  ( void ) pvParameters;

  for (;;){ // A Task shall never return or exit.
    xSemaphoreTake(SPImutex, portMAX_DELAY); // enter critical section
    SPI.begin(SCLK, MISO, MOSI, EXT_ADC_CS);
    extADCresult = AD7193.singleConversion();
    SPI.end();
    xSemaphoreGive(SPImutex); // exit critical section
    vTaskDelay(250/portTICK_PERIOD_MS);
  }
}

void TaskIntAnalogRead(void *pvParameters)
{
  ( void ) pvParameters;
  for (;;)
  {
    digitalWrite(VIN_LVL_EN_PIN, HIGH);
    vTaskDelay(10);  // Delay to settle LPF
    // read the input on analog pin:
    float sensorValue = analogRead(VIN_LVL_PIN)*VIN_LVL_COUNTS_TO_V;
    char vBuffer[6];
    dtostrf(sensorValue,4,2, vBuffer);  // Cannot use printf with floats with small task sizes (<2048) - so do this instead
    digitalWrite(VIN_LVL_EN_PIN, LOW);
    // print out the value you read:
    //Serial.printf("Vin: %s\t", vBuffer);
    sensorValue = analogRead(PWR_5V_A_LVL_PIN)*PWR_5V_LVL_COUNTS_TO_V;
    dtostrf(sensorValue,4,2, vBuffer);
    //Serial.printf("5V_A: %s\n", vBuffer);
    vTaskDelay(3000/portTICK_PERIOD_MS); // 30s delay
  }
}

void TaskPowerZeroButton(void *pvParameters)
{ 
  static uint8_t powerButtonCounter = 0;

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
          Serial.printf("Long power/zero button press.\n");
          // While user has button held down, don't do anything else
          while(digitalRead(PWR_ZERO_BTN))
          {
            vTaskDelay(250/portTICK_PERIOD_MS);
          }
        }
        else
        {
          // Short press detected
          powerButtonStat = short_press;
          powerButtonFlag = short_press_flag;
          Serial.printf("Short power/zero button press.\n");
        }
      }
      else
      {
        // Fresh button press event detected
        powerButtonStat = is_pressed;
      }
    }
    else
    {
      powerButtonStat = no_press;
      powerButtonCounter = 0;
    }

    vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS);
  }
}

void TaskUnitButton(void *pvParameters)
{  
  static uint8_t unitButtonCounter = 0;

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
          Serial.printf("Long unit button press.\n");
          // While user has button held down, don't do anything else
          while(!digitalRead(UNIT_BTN))
          {
            vTaskDelay(250/portTICK_PERIOD_MS);
          }
        }
        else
        {
          // Short press detected
          unitButtonStat = short_press;
          unitButtonFlag = short_press_flag;
          Serial.printf("Short unit button press.\n");
        }
      }
      else
      {
        // Fresh button press event detected
        unitButtonStat = is_pressed;
      }
    }
    else
    {
      unitButtonStat = no_press;
      unitButtonCounter = 0;
    }
    vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS);
  }
}

void TaskAuxButton(void *pvParameters)
{  
  static uint8_t auxButtonCounter = 0;

  ( void ) pvParameters;

  for (;;)
  {
    
    // Check aux button
    if(!digitalRead(AUX_BTN))
    {
      if(auxButtonStat == is_pressed)
      {
        // Button is still pressed after debounce period so delay for a period of time
        while((!digitalRead(AUX_BTN)) && auxButtonCounter < LONG_PRESS_COUNT_THRESHOLD)
        {
          vTaskDelay(LONG_PRESS_LOOP_DELAY/portTICK_PERIOD_MS);
          auxButtonCounter++;
        }
        if(auxButtonCounter >= LONG_PRESS_COUNT_THRESHOLD)
        {
          // Long-press detected
          auxButtonStat = long_press;
          auxButtonFlag = long_press_flag;
          Serial.printf("Long aux button press.\n");
          // While user has button held down, don't do anything else
          while(!digitalRead(AUX_BTN))
          {
            vTaskDelay(250/portTICK_PERIOD_MS);
          }
        }
        else
        {
          // Short press detected
          auxButtonStat = short_press;
          auxButtonFlag = short_press_flag;
          Serial.printf("Short aux button press.\n");
        }
      }
      else
      {
        // Fresh button press event detected
        auxButtonStat = is_pressed;
      }
    }
    else
    {
      auxButtonStat = no_press;
      auxButtonCounter = 0;
    }
    vTaskDelay(DEBOUNCE_TIME/portTICK_PERIOD_MS);
  }
}

void TaskUpdateWeightReadingLCD(void *pvParameters)
{
  ( void ) pvParameters;

  for(;;)
  {
    if(updateLCDWeight)
    {
      String s_extADCweight;

      extADCweight = (extADCresult - zeroValue)/calValue  - tareValue;
      if(calUnit == lb && unitVal == kg) extADCweight = extADCweight / kgtolbScalar;
      else if(calUnit == kg && unitVal == lb) extADCweight = extADCweight * kgtolbScalar;
      extADCweight = removeNegativeSignFromZero(extADCweight);

      //char vBuffer[7];
      //dtostrf(extADCweight,5,2, vBuffer);  // Cannot use printf with floats with small task sizes (<2048) - so do this instead
      //Serial.printf("ExtADC: %s\n", vBuffer);

      u8g2.clearBuffer();
      u8g2.setFont(WVAL_FONT);
      s_extADCweight = String(extADCweight, WVAL_DEC_PLS);
      // Split string at the decimal point because display looks strange with large decimal point spacing

      // Print decimal point first
      u8g2.setCursor(WVAL_X_POS-u8g2.getStrWidth(s_extADCweight.substring(s_extADCweight.length()-WVAL_DEC_PLS).c_str())-DEC_PT_S_PX+((DEC_PT_S_PX-DEC_PT_W_PX)/2)-DEC_PT_W_PX, WVAL_Y_POS);
      u8g2.print(".");

      // Print numbers left of the deicmal place
      u8g2.setCursor(WVAL_X_POS-u8g2.getStrWidth((s_extADCweight.c_str()+1))-DEC_PT_S_PX, WVAL_Y_POS);
      u8g2.print(s_extADCweight.substring(0, s_extADCweight.length()-WVAL_DEC_PLS-1).c_str());  
      
      // Print numbers right of the decimal place
      u8g2.setCursor(WVAL_X_POS-u8g2.getStrWidth(s_extADCweight.substring(s_extADCweight.length()-WVAL_DEC_PLS).c_str()), WVAL_Y_POS);
      u8g2.print(s_extADCweight.substring(s_extADCweight.length()-WVAL_DEC_PLS).c_str());

      u8g2.setFont(UNIT_FONT);
      u8g2.setCursor(UNIT_X_POS, UNIT_Y_POS);
      
      u8g2.print(unitAbbr[unitVal]);
      sendBufferSPISafe();
    }
    vTaskDelay(250/portTICK_PERIOD_MS);
  }
}

void drawLogo(void)
{
  u8g2.drawXBM(6, 12, penner_logo_width, penner_logo_height, penner_logo_bits);
}

void doCalibration()
{
  Serial.println("Turning off weight update flag.");
  updateLCDWeight = false;
  vTaskDelay(500/portTICK_PERIOD_MS); // Delay to allow LCD update task to finish and read updateLCDWeight flag
  u8g2.clearBuffer();
  u8g2.setFont(MSG_FONT);
  u8g2.setCursor(0, 20);
  u8g2.print("--Scale Calibration--"); 
  u8g2.setCursor(0, 30);
  u8g2.print("Press ZERO to cont.");
  u8g2.setCursor(0, 40);
  u8g2.print("Press UNIT to exit.");
  sendBufferSPISafe();

  // Allow user to release buttons before proceeding
  while (powerButtonFlag != no_press_flag || unitButtonFlag != no_press_flag)
  {
    vTaskDelay(25/portTICK_PERIOD_MS);
  }

  // Reset button flags
  powerButtonFlag = no_press_flag;
  unitButtonFlag = no_press_flag;

  // Wait for user to press a button
  while (powerButtonFlag == no_press_flag && unitButtonFlag == no_press_flag)
  {
    vTaskDelay(25/portTICK_PERIOD_MS);
  }

  if(unitButtonFlag != no_press_flag)
  {
    Serial.printf("Calibration aborted.\n");
    // Reset button flags
    powerButtonFlag = no_press_flag;
    unitButtonFlag = no_press_flag;
    Serial.println("Setting weight update flag.");
    updateLCDWeight = true;   
    return;
  }

  // Reset button flags
  powerButtonFlag = no_press_flag;
  unitButtonFlag = no_press_flag;  

  // Set calibration units
  calUnit = unitVal;
  u8g2.clearBuffer();
  u8g2.setCursor(0, 20);
  u8g2.print("Press UNIT to select"); 
  u8g2.setCursor(0, 30);
  u8g2.printf("calibration units: %s", unitAbbr[calUnit]);
  u8g2.setCursor(0, 40);
  u8g2.print("Press ZERO to cont.");
  sendBufferSPISafe();

  // Allow user to release ZERO before proceeding
  while (powerButtonFlag != no_press_flag)
  {
    vTaskDelay(25/portTICK_PERIOD_MS);
  }

  // Wait for user to press ZERO
  while (powerButtonFlag == no_press_flag)
  {
    // Check unit button flag
    if (unitButtonFlag == short_press_flag)
    {
      // Change cal units
      Serial.printf("Change cal unit flag set.\n");
      unitButtonFlag = no_press_flag;
      if(calUnit == kg) calUnit = lb;
      else if(calUnit == lb) calUnit = kg; 
      u8g2.setCursor(0, 30);
      u8g2.printf("calibration units: %s", unitAbbr[calUnit]); 
      sendBufferSPISafe();
    }

    vTaskDelay(25/portTICK_PERIOD_MS);
  }

  // Reset button flag
  powerButtonFlag = no_press_flag;

  // Set calibration weight
  u8g2.clearBuffer();
  u8g2.setCursor(0, 20);
  u8g2.print("Press * / UNIT to set");
  u8g2.setCursor(0, 30);  
  u8g2.printf("cal. weight: %5u %s", calWeight, unitAbbr[calUnit]);
  u8g2.setCursor(0, 40);
  u8g2.print("Press ZERO to cont.");
  sendBufferSPISafe();

// Wait for user to press ZERO
  while (powerButtonFlag == no_press_flag)
  {
    if (auxButtonStat)
    {
      if (auxButtonStat == long_press)
      {
        // User is holding down the button
        if (calWeight < (MAX_CAL_VAL-10)) calWeight += 10;
        else calWeight = MAX_CAL_VAL;
      }
      else
      {
        if (calWeight < (MAX_CAL_VAL)) calWeight ++;
      } 
      u8g2.setCursor(0, 30);  
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
      u8g2.setCursor(0, 30);  
      u8g2.printf("cal. weight: %5u %s", calWeight, unitAbbr[calUnit]); 
      sendBufferSPISafe();
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
  }

  // Commit calWeight and calUnit to EEPROM
  EEPROM.put(calWeight_eepromAdress, calWeight);
  EEPROM.put(calUnit_eepromAdress, calUnit);  
  EEPROM.commit();

  // Reset button flags
  powerButtonFlag = no_press_flag;
  unitButtonFlag = no_press_flag;
  auxButtonFlag = no_press_flag;

  u8g2.clearBuffer();
  u8g2.setCursor(0, 20);
  u8g2.print("--Scale Calibration--");
  u8g2.setCursor(0, 30);  
  u8g2.print("Unload scale then");
  u8g2.setCursor(0, 40);
  u8g2.print("press ZERO.");
  sendBufferSPISafe();
  while (powerButtonFlag == no_press_flag)
  {
    vTaskDelay(25/portTICK_PERIOD_MS);
  }

  // Reset button flags
  powerButtonFlag = no_press_flag;
  unitButtonFlag = no_press_flag;

  u8g2.clearBuffer();
  u8g2.setCursor(0, 20);
  u8g2.print("--Scale Calibration--");  
  u8g2.setCursor(0, 30);
  u8g2.printf("Wait.");
  sendBufferSPISafe();
  delay(1000);
  u8g2.setCursor(0, 30);
  u8g2.printf("Wait..");
  sendBufferSPISafe();
  delay(1000);
  u8g2.setCursor(0, 30);
  u8g2.printf("Wait...");
  sendBufferSPISafe();
  delay(1000);

  // Store zero
  zeroValue = extADCresult;

  u8g2.clearBuffer();
  u8g2.setCursor(0, 20);
  u8g2.print("--Scale Calibration--");
  u8g2.setCursor(0, 30);  
  u8g2.printf("Load %d %s then press", calWeight, unitAbbr[calUnit]);
  u8g2.setCursor(0, 40);
  u8g2.print("ZERO.");
  sendBufferSPISafe();
  while (powerButtonFlag == no_press_flag)
  {
    vTaskDelay(25/portTICK_PERIOD_MS);
  }

  // Reset button flags
  powerButtonFlag = no_press_flag;
  unitButtonFlag = no_press_flag;
  
  u8g2.clearBuffer();
  u8g2.setCursor(0, 20);
  u8g2.print("--Scale Calibration--");  
  u8g2.setCursor(0, 30);
  u8g2.printf("Wait.");
  sendBufferSPISafe();
  delay(1000);
  u8g2.setCursor(0, 30);
  u8g2.printf("Wait..");
  sendBufferSPISafe();
  delay(1000);
  u8g2.setCursor(0, 30);
  u8g2.printf("Wait...");
  sendBufferSPISafe();
  delay(1000);  

  // Store span
  calValue = ((float)extADCresult - (float)zeroValue)/(float)calWeight;  

  Serial.printf("extADCresult: %d\n", extADCresult);
  Serial.printf("zeroValue: %d\n", zeroValue);
  Serial.printf("calWeight: %u\n", calWeight);
  Serial.printf("calValue: %f\n", calValue);
  Serial.printf("calUnit: %s\n", unitAbbr[calUnit]);  

  u8g2.clearBuffer();
  u8g2.setCursor(0, 20);
  u8g2.print("Cal done. Press ZERO.");  
  u8g2.setCursor(0, 30);  
  u8g2.printf("Zero: %d\n\r", zeroValue);
  u8g2.setCursor(0, 40);
  u8g2.printf("Span: %f\n\r", calValue);  
  sendBufferSPISafe();
  while (powerButtonFlag == no_press_flag)
  {
    vTaskDelay(25/portTICK_PERIOD_MS);
  }  

  // Newly calibrated scale - do not need tare
  tareValue = 0;
  EEPROM.put(calVal_eepromAdress, calValue);
  EEPROM.put(calUnit_eepromAdress, calUnit);
  EEPROM.put(zeroVal_eepromAdress, zeroValue);
  EEPROM.commit();

  // Reset button flags
  powerButtonFlag = no_press_flag;
  unitButtonFlag = no_press_flag;
  Serial.println("Setting weight update flag.");
  updateLCDWeight = true;
}

void sendBufferSPISafe(void)
{
  if (xSemaphoreTake(SPImutex, ( TickType_t ) 25) == pdTRUE) // enter critical section
  {
    SPI.begin(SCLK, MISO, MOSI, LCD_CS);
    u8g2.initInterface();
    u8g2.sendBuffer();
    SPI.end();
    xSemaphoreGive(SPImutex); // exit critical section
  }
  else
  {
    Serial.println("Couldn't take SPI mutex.");
  }
}

void powerDown(void)
{
  updateLCDWeight = false;
  EEPROM.commit();  // Save our settings to EEPROM
  // Disable LCD backlight, clear LCD, power down
  digitalWrite(LCD_BACKLIGHT, LOW);
  u8g2.clearBuffer();
  sendBufferSPISafe();
  u8g2.setPowerSave(true);
  vTaskDelay(500/portTICK_PERIOD_MS);
  while(digitalRead(PWR_ZERO_BTN));
  digitalWrite(V3V3_EN, LOW);
  for (;;);
}

float removeNegativeSignFromZero(float weightValue)
{
  float precision = (float) WVAL_PREC / ((float)WVAL_DEC_PLS * 10);
  if((weightValue < precision) && (weightValue > -precision)) weightValue = 0;
  return weightValue;
}
