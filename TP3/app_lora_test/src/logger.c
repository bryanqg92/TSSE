/************************************************************************************************
Copyright (c) 2025, Leandro Quiroga <bryanqg92@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

SPDX-License-Identifier: MIT
*************************************************************************************************/

/** @file logger.c
 ** @brief 
 **/

/* === Headers files inclusions =============================================================== */
#include "logger.h"
#include <stdio.h>
#include <stdarg.h>

/* === Macros definitions ====================================================================== */

/* === Private data type declarations ========================================================== */
static uint16_t *port_adress;
/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */
static log_level_t current_level = LOG_LEVEL_INFO;
/* === Private function implementation ========================================================= */

/* === Public function implementation ========================================================== */

void logger_init(log_level_t level) {
    current_level = level;
}

void logger_log(log_level_t level, const char* tag, const char* format, ...) {
    if (level > current_level) return;

    const char* level_str[] = { "ERROR", "WARN", "INFO", "DEBUG" };
    FILE* out = (level <= LOG_LEVEL_WARN) ? stderr : stdout;

    fprintf(out, "[%s] [%s] ", level_str[level], tag);

    va_list args;
    va_start(args, format);
    vfprintf(out, format, args);
    va_end(args);

    fprintf(out, "\n");
}

/* === End of documentation ==================================================================== */
