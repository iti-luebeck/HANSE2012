#ifndef I2CREG_H
#define I2CREG_H

#include "i2ccomm.h"
#include "watchdog.h"
#include "error.h"

class i2creg {
public:
    static void write_reg8(uint8_t addr, uint8_t reg, uint8_t val) {
	while (true) {
	    i2ccomm::start();
	    i2ccomm::write_byte(addr);
	    i2ccomm::write_byte(reg);
	    i2ccomm::write_byte(val);
	    i2ccomm::finish();
	    if (i2ccomm::error() == i2c_write_nack) {
		i2ccomm::setup();
		continue;
	    }
	    break;
 	}
	//	i2ccomm::stop(); // confuses md22s
	if (i2ccomm::error())
	    error_condition = i2ccomm::error();
	else
	    watchdog::i2c_progress();
    }

    static uint8_t read_reg8(uint8_t addr, uint8_t reg) {
	while (true) {
	    i2ccomm::start();
	    i2ccomm::write_byte(addr);
	    i2ccomm::write_byte(reg);
	    i2ccomm::start();
	    i2ccomm::write_byte(addr + 1);
	    i2ccomm::enqueue_read(true);
	    i2ccomm::finish();
	    if (i2ccomm::error() == i2c_write_nack ||
		i2ccomm::error() == i2c_read_nack) {
		i2ccomm::setup();
		continue;
	    }
	    break;
	}
	//	i2ccomm::stop(); // confuses md22s
	uint8_t b = i2ccomm::read_byte();
	if (i2ccomm::error())
	    error_condition = i2ccomm::error();
	else
	    watchdog::i2c_progress();
	return b;
    }

    static uint16_t read_reg16(uint8_t addr, uint8_t reg) {
	while (true) {
	    i2ccomm::start();
	    i2ccomm::write_byte(addr);
	    i2ccomm::write_byte(reg);
	    i2ccomm::start();
	    i2ccomm::write_byte(addr + 1);
	    i2ccomm::enqueue_read(false);
	    i2ccomm::enqueue_read(true);
	    i2ccomm::finish();
	    if (i2ccomm::error() == i2c_write_nack ||
		i2ccomm::error() == i2c_read_nack) {
		i2ccomm::setup();
		continue;
	    }
	    break;
	}
	//	i2ccomm::stop(); // confuses md22s
	uint8_t bh = i2ccomm::read_byte();
	uint8_t bl = i2ccomm::read_byte();
	if (i2ccomm::error())
	    error_condition = i2ccomm::error();
	else
	    watchdog::i2c_progress();
	return (bh << 8) | bl;
    }
};

#endif
