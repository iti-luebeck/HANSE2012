#ifndef ERROR_H
#define ERROR_H

#include "stdint.h"

enum error_codes
{
    no_error = 0, // a
    unknown_error, // b
    checksum_error, // c
    // serial error codes
    serial_abort, // d
    serial_receive_error, // e
    serial_receive_overflow, // f
    // i2c error codes
    i2c_abort, // g
    i2c_arb_lost, // h
    i2c_write_nack, // i
    i2c_read_nack, // j
    i2c_receive_overflow, // k
};

typedef uint8_t error_code;

extern error_code error_condition;

#endif
