/*
 * debug_log.cpp — RAM ring-buffer implementation for debug messages
 */

#include "debug_log.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

static char log_buf[DBG_LOG_BUF_SIZE];
static size_t log_head  = 0;  // next write position
static size_t log_count = 0;  // bytes currently in buffer
static SemaphoreHandle_t log_mutex = NULL;

void dbg_log_init(void) {
    log_mutex = xSemaphoreCreateMutex();
    memset(log_buf, 0, sizeof(log_buf));
}

static void log_write(const char *data, size_t len) {
    if (!log_mutex) return;
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    for (size_t i = 0; i < len; i++) {
        log_buf[log_head] = data[i];
        log_head = (log_head + 1) % DBG_LOG_BUF_SIZE;
        if (log_count < DBG_LOG_BUF_SIZE) log_count++;
    }
    xSemaphoreGive(log_mutex);
}

void dbg_printf(const char *fmt, ...) {
    char tmp[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    if (n > 0) {
        if ((size_t)n > sizeof(tmp) - 1) n = sizeof(tmp) - 1;
        log_write(tmp, (size_t)n);
    }
}

void dbg_println(const char *msg) {
    size_t len = strlen(msg);
    log_write(msg, len);
    log_write("\n", 1);
}

size_t dbg_read(char *out, size_t maxlen) {
    if (!log_mutex) return 0;
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    size_t to_copy = (log_count < maxlen) ? log_count : maxlen;
    size_t start = (log_head + DBG_LOG_BUF_SIZE - log_count) % DBG_LOG_BUF_SIZE;
    for (size_t i = 0; i < to_copy; i++) {
        out[i] = log_buf[(start + i) % DBG_LOG_BUF_SIZE];
    }
    xSemaphoreGive(log_mutex);
    return to_copy;
}

void dbg_clear(void) {
    if (!log_mutex) return;
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    log_head  = 0;
    log_count = 0;
    xSemaphoreGive(log_mutex);
}
