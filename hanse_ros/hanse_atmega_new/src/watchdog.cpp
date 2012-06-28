#include "watchdog.h"
#include "serial.h"
#include "i2ccomm.h"

watchdog watchdog::data;

ISR(TIMER1_OVF_vect)
{
    watchdog::irq();
}

void watchdog::irq()
{
    data.serial_counter++;
    data.i2c_counter++;
    if (data.serial_counter > io_watchdog_serial_limit) {
	serial::abort();
	data.serial_counter = 0;
    }
    if (data.i2c_counter > io_watchdog_i2c_limit) {
	i2ccomm::abort();
	data.i2c_counter = 0;
    }
    timer1::reset_timer(~io_watchdog_timer);
}
