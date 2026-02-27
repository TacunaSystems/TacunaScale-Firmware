#ifndef APPCONFIG_H
#define APPCONFIG_H

/*
 * appconfig.h — shared definitions used by PennerScale.cpp and scpi_interface.cpp
 */

#define FW_VER  "1.1.0b"

// FreeRTOS diagnostic SCPI commands (SYST:DIAG:STATS? / SYST:DIAG:LIST?)
#ifndef FREERTOS_DIAG
#define FREERTOS_DIAG 0
#endif

// LCD backlight
#define LCD_BACKLIGHT         14
#define BACKLIGHT_PWM_DEFAULT 31  // percent (31% ≈ 80/255)

// Unit enumerations
enum e_backlightEnable {off = 0, on = 1, on_motion = 2};
enum e_unitVal {kg = 0, lb = 1};

// EEPROM address map (shared between PennerScale.cpp and scpi_interface.cpp)
#define EEPROM_ADDR_CAL_VALUE    0
#define EEPROM_ADDR_ZERO_VALUE   (EEPROM_ADDR_CAL_VALUE  + (int)sizeof(float))
#define EEPROM_ADDR_BACKLIGHT    (EEPROM_ADDR_ZERO_VALUE + (int)sizeof(int32_t))
#define EEPROM_ADDR_UNIT_VAL     (EEPROM_ADDR_BACKLIGHT  + (int)sizeof(e_backlightEnable))
#define EEPROM_ADDR_CAL_WEIGHT   (EEPROM_ADDR_UNIT_VAL   + (int)sizeof(e_unitVal))
#define EEPROM_ADDR_CAL_UNIT     (EEPROM_ADDR_CAL_WEIGHT + (int)sizeof(uint32_t))
#define EEPROM_ADDR_WEIGHT_MAX   (EEPROM_ADDR_CAL_UNIT   + (int)sizeof(e_unitVal))
#define EEPROM_ADDR_BACKLIGHT_PWM (EEPROM_ADDR_WEIGHT_MAX + (int)sizeof(float))
#define EEPROM_ADDR_ECHO          (EEPROM_ADDR_BACKLIGHT_PWM + (int)sizeof(uint8_t))
#define EEPROM_ADDR_PROMPT        (EEPROM_ADDR_ECHO          + (int)sizeof(uint8_t))

// Debug logging — messages go to a RAM ring buffer readable via SYST:LOG?
// Set SCPI_DEBUG to 0 to compile out all debug logging.
#ifndef SCPI_DEBUG
#define SCPI_DEBUG 1
#endif

#if SCPI_DEBUG
  #include "debug_log.h"
  #define DBG_PRINTF(fmt, ...)  dbg_printf(fmt, ##__VA_ARGS__)
  #define DBG_PRINTLN(msg)      dbg_println(reinterpret_cast<const char*>(msg))
#else
  #define DBG_PRINTF(fmt, ...)  ((void)0)
  #define DBG_PRINTLN(msg)      ((void)0)
#endif

#endif // APPCONFIG_H
