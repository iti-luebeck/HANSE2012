#ifndef I2CCOMM_H
#define I2CCOMM_H

#include <avr/io.h>
#include <avr/sleep.h>
#include <util/atomic.h>

#include "io_config.h"
#include "i2c.h"
#include "sleep.h"
#include "error.h"

// single master, avr master only i2c communication
class i2ccomm {
    static i2ccomm data;

    static constexpr uint8_t cmd_start = 0;
    static constexpr uint8_t cmd_write = 1;
    static constexpr uint8_t cmd_read = 2;
    static constexpr uint8_t cmd_last_read = 3;
    static constexpr uint8_t cmd_stop = 4;


    static constexpr unsigned buffer_bits = 6;
    static constexpr unsigned buffer_size = 1 << buffer_bits;
    static constexpr uint8_t buffer_mask = buffer_size - 1;

    volatile uint8_t rx_buffer[buffer_size];
    volatile uint8_t tx_buffer[buffer_size];

    volatile uint8_t tx_write;
    volatile uint8_t tx_read;
    volatile uint8_t rx_write;
    volatile uint8_t rx_read;

    volatile error_code error_condition;

    volatile bool reading;
    volatile bool nacked;
    volatile bool ready_;

    static bool check_errors() {
	bool err = false;

	uint8_t status = i2c::status();

	switch (status) {
	case TW_START: break;
	case TW_REP_START: break;
	case TW_MT_SLA_ACK: break;
	case TW_MT_SLA_NACK: data.nacked = true; break;
	case TW_MT_DATA_ACK: break;
	case TW_MT_DATA_NACK: data.nacked = true; break;
	case TW_MT_ARB_LOST: err = true; break;
	case TW_MR_SLA_ACK: break;
	case TW_MR_SLA_NACK: data.nacked = true; break;
	case TW_MR_DATA_ACK: break;
	case TW_MR_DATA_NACK: data.nacked = true; break;
	case TW_NO_INFO: break;
	default: err = true; break;
	}

	if (err)
	    data.error_condition = i2c_arb_lost;
	return err;
    }

    static void enqueue_command(uint8_t command) {
	 uint8_t w, r;
	 w = data.tx_write;
	 r = data.tx_read;
	 data.tx_buffer[w] = command;
	 w = (w + 1) & buffer_mask;
	 while (w == r) {
	     sleep::wait_for_irq();
	     if (data.error_condition)
		 return;
	     r = data.tx_read;
	 }
	 data.tx_write = w;
	 //	 i2c::irq_en(true);
    }

    static bool ready() {
	if (data.ready_) {
	    data.ready_ = false;
	    return true;
	}
	return i2c::ready();
    }

public:
    static void setup() {
	i2c::setup(i2c_freq);
	i2c::enable(true);
	data.tx_write = 0;
	data.tx_read = 0;
	data.rx_write = 0;
	data.rx_read = 0;
	data.error_condition = no_error;
	data.reading = false;
	data.nacked = false;
	data.ready_ = true;
    }

    static void start() {
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
	    if (data.error_condition)
		return;
	    if (i2ccomm::ready()) {
		if (check_errors())
		    return;
		i2c::start(true);
		data.nacked = false;
		return;
	    }
	    enqueue_command(cmd_start);
	}
    }

    static void write_byte(uint8_t byte) {
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	    if (data.error_condition)
		return;
	    if (i2ccomm::ready()) {
		if (check_errors())
		    return;
		if (data.nacked) {
		    data.error_condition = i2c_write_nack;
		    return;
		}
		i2c::tx_byte(byte,true);
		return;
	    }
	    uint8_t w, r;
	    w = data.tx_write;
	    r = data.tx_read;
	    data.tx_buffer[w] = cmd_write;
	    w = (w + 1) & buffer_mask;
	    while (w == r) {
		sleep::wait_for_irq();
		if (data.error_condition)
		    return;
		r = data.tx_read;
	    }
	    data.tx_buffer[w] = byte;
	    w = (w + 1) & buffer_mask;
	    while (w == r) {
		sleep::wait_for_irq();
		if (data.error_condition)
		    return;
		r = data.tx_read;
	    }
	    // it is important that we write back only after enqueing
	    // the whole command!
	    data.tx_write = w;
	    //	    i2c::irq_en(true);
	}
    }

    static void enqueue_read(bool last = false) {
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
	    if (data.error_condition)
		return;
	    if (i2ccomm::ready()) {
		if (check_errors())
		    return;
		if (data.nacked) {
		    data.error_condition = i2c_read_nack;
		    return;
		}
		i2c::prepare_rx_byte(!last, true);
		data.reading = true;
		return;
	    }
	    enqueue_command(last ? cmd_last_read : cmd_read);
	}
    }

    static void stop() {
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	    if (data.error_condition)
		return;
	    if (i2ccomm::ready()) {
		if (check_errors())
		    return;
		i2c::stop(true);
		data.ready_ = true;
		return;
	    }
	    enqueue_command(cmd_stop);
	}
    }

    static void finish() {
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	    if (data.error_condition)
		return;
	    if (!i2ccomm::ready()) {
		sleep::wait_for_irq();
	    }
	}
    }

    // needs to have a preceding enqueue_read() !
    static uint8_t read_byte() {
        uint8_t b;
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            uint8_t w, r;
            while (true) {
		if (data.error_condition)
		    return 0;
                w = data.rx_write;
                r = data.rx_read;
                if (w == r) {
                    // buffer is empty wait for an interrupt and retry;
		    sleep::wait_for_irq();
                } else {
                    break;
                }
            }
            b = data.rx_buffer[r];
            data.rx_read = (r + 1) & buffer_mask;
        }
        return b;
    }

    static error_code error() { return data.error_condition; }

    static void abort() { data.error_condition = i2c_abort; }

    static void irq();
};


#endif
