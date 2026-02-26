#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

/*
 * debug_log.h — RAM ring-buffer for debug messages, readable via SCPI
 *
 * Replaces Serial debug prints so the UART carries only clean SCPI traffic.
 * Messages are stored in a fixed-size ring buffer; when full, oldest data
 * is silently overwritten.
 */

#include <stddef.h>

#define DBG_LOG_BUF_SIZE 2048

void dbg_log_init(void);
void dbg_printf(const char *fmt, ...);
void dbg_println(const char *msg);
size_t dbg_read(char *out, size_t maxlen);
void dbg_clear(void);

#endif // DEBUG_LOG_H
