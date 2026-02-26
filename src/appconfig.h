#ifndef APPCONFIG_H
#define APPCONFIG_H

/*
 * appconfig.h — shared definitions used by PennerScale.cpp and scpi_interface.cpp
 */

#define FW_VER  "1.0.0"

// FreeRTOS diagnostic SCPI commands (SYST:DIAG:STATS? / SYST:DIAG:LIST?)
#ifndef FREERTOS_DIAG
#define FREERTOS_DIAG 1
#endif

// LCD backlight pin (needed by SCPI backlight command)
#define LCD_BACKLIGHT 14

// Unit enumerations
enum e_backlightEnable {off = 0, on = 1, on_motion = 2};
enum e_unitVal {kg = 0, lb = 1};

// Debug output macros — set SCPI_DEBUG to 0 to silence Serial debug prints
// so the UART carries only clean SCPI traffic.
#ifndef SCPI_DEBUG
#define SCPI_DEBUG 1
#endif

#if SCPI_DEBUG
  #define DBG_PRINTF(fmt, ...)  Serial.printf(fmt, ##__VA_ARGS__)
  #define DBG_PRINTLN(msg)      Serial.println(msg)
#else
  #define DBG_PRINTF(fmt, ...)  ((void)0)
  #define DBG_PRINTLN(msg)      ((void)0)
#endif

#endif // APPCONFIG_H
