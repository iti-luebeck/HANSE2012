/*
 * Logger.h
 *
 *  Created on: 04.06.2011
 *      Author: forouher
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <avr/pgmspace.h>

#include <stdio.h>

class Logger {

public:
	static void fatal(const char* format, ...);
	static void error(const char* format, ...);
	static void warn(const char* format, ...);
	static void info(const char* format, ...);
	static void debug(const char* format, ...);

private:
	static char log_buffer[];
	static const uint8_t log_buffer_len;

};

#endif /* LOGGER_H_ */
