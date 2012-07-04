#ifndef SERIAL_H
#define SERIAL_H

#include "io_config.h"
#include <avr/sleep.h>
#include <util/atomic.h>
#include "sleep.h"
#include "error.h"

class serial {
    static serial data;

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

public:
    static void setup() {
	serial_port.set_settings(usart::settings(baudrate));
	data.tx_write = 0;
	data.tx_read = 0;
	data.rx_write = 0;
	data.rx_read = 0;
        data.error_condition = no_error;
        serial_port.rx_irq_en(true);
        serial_port.tx_ready_irq_en(false);
        serial_port.rx_en(true);
        serial_port.tx_en(true);
    }

    // Assumes interrupts enabled
    static void write_byte(uint8_t byte) {
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
	    if (data.error_condition)
		return;
            bool ready = serial_port.tx_ready();
            if (ready) {
                serial_port.tx_byte(byte);
                return;
            }
            uint8_t w, r;
            w = data.tx_write;
            r = data.tx_read;
            data.tx_buffer[w] = byte;
            w = (w + 1) &  buffer_mask;
            while (w == r) {
		// buffer is full wait for a byte to be transmitted
		sleep::wait_for_irq();
		if (data.error_condition)
		    return;
		r = data.tx_read;
            }
            data.tx_write = w;
            serial_port.tx_ready_irq_en(true);
        }
    }

    // Assumes interrupts enabled, blocks
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

    static void finish() {
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	    if (data.error_condition)
		return;
	    if (!serial_port.tx_ready()) {
		sleep::wait_for_irq();
	    }
	}
    }

    static error_code error() {
	return data.error_condition;
    }

    static void abort() { data.error_condition = serial_abort; }

    static void rx_irq();
    static void tx_irq();
};



#endif
