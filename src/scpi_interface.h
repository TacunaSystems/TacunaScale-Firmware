#ifndef SCPI_INTERFACE_H
#define SCPI_INTERFACE_H

/*
 * scpi_interface.h — SCPI parser configuration and task declaration
 */

#include <scpi/scpi.h>

// SCPI buffer sizes
#define SCPI_INPUT_BUFFER_LENGTH  256
#define SCPI_ERROR_QUEUE_SIZE     17

// IDN fields: Manufacturer, Model, Serial, FW version
#define SCPI_IDN1  "Penner"
#define SCPI_IDN2  "BathingScale"
#define SCPI_IDN3  "00000000"
#define SCPI_IDN4  FW_VER

// Global SCPI context (defined in scpi_interface.cpp)
extern scpi_t scpi_context;

// FreeRTOS task
void TaskSCPI(void *pvParameters);

#endif // SCPI_INTERFACE_H
