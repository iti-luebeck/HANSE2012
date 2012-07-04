#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <avr/wdt.h>

#include "timer1.h"
#include <util/atomic.h>
#include "io_config.h"

class watchdog {
    uint8_t serial_counter;
    uint8_t i2c_counter;
    uint8_t saved_mcusr;
    static watchdog data;
public:
    static void setup(bool initial)
    {
	if (initial) {
	    data.saved_mcusr = MCUSR;
	    MCUSR = 0;
	}
	timer1::setup();
	timer1::reset_timer(~io_watchdog_timer);
	timer1::overflow_irq_en(true);
	data.serial_counter = 0;
	data.i2c_counter = 0;
	if (initial)
	    wdt_enable(WDTO_4S);
    }

    static void serial_progress()
    {
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	    data.serial_counter = 0;
	}
    }

    static void i2c_progress()
    {
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
	    data.i2c_counter = 0;
	}
    }

    static void progress()
    {
	wdt_reset();
    }

    static bool was_intended_reset()
    {
	return !(data.saved_mcusr & (1<<WDRF));
    }

    static void irq();
};


#endif
