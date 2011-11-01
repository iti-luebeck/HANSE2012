/*
 * Logger class.
 *
 * Log messages are sent to the PC using the rosserial interface.
 *
 * Note: argument format is expected to be in program memory (to save
 * precious SRAM), so the string has to wrapped inside a PSTR("") macro.
 *
 * Example:
 *
 * Logger::debug(PSTR("All quiet on the western front, foo=%i"), bar_int);
 *
 *  Created on: 04.06.2011
 *      Author: forouher
 */

#include "Logger.h"
#include "stdio.h"

#include <ros.h>

extern ros::NodeHandle node;

const uint8_t Logger::log_buffer_len = 80;
char Logger::log_buffer[Logger::log_buffer_len];

void Logger::fatal(const char* format, ...) {
    va_list ap;
    va_start(ap, format);
    vsnprintf_P(log_buffer, Logger::log_buffer_len, format, ap);
    va_end(ap);
    node.logfatal(log_buffer);
}

void Logger::error(const char* format, ...) {
    va_list ap;
    va_start(ap, format);
    vsnprintf_P(log_buffer, Logger::log_buffer_len, format, ap);
    va_end(ap);
    node.logerror(log_buffer);
}

void Logger::warn(const char* format, ...) {
    va_list ap;
    va_start(ap, format);
    vsnprintf_P(log_buffer, Logger::log_buffer_len, format, ap);
    va_end(ap);
    node.logwarn(log_buffer);
}

void Logger::info(const char* format, ...) {
    va_list ap;
    va_start(ap, format);
    vsnprintf_P(log_buffer, Logger::log_buffer_len, format, ap);
    va_end(ap);
    node.loginfo(log_buffer);
}

void Logger::debug(const char* format, ...) {
    va_list ap;
    va_start(ap, format);
    vsnprintf_P(log_buffer, Logger::log_buffer_len, format, ap);
    va_end(ap);
    node.logdebug(log_buffer);
}
