#include "i2ccomm.h"

i2ccomm i2ccomm::data;

ISR(TWI_vect) { i2ccomm::irq(); }

void i2ccomm::irq() {
    if (data.error_condition || check_errors()) {
	i2c::irq_en(false);
	i2c::enable(false);
	return;
    }
    if (data.reading) {
	data.reading = false;
	uint8_t w, r;
	w = data.rx_write;
	r = data.rx_read;
	data.rx_buffer[w] = i2c::rx_byte();
	w = (w + 1) & buffer_mask;
	if (w == r) {
	    data.error_condition = i2c_receive_overflow;
	    i2c::irq_en(false);
	    i2c::enable(false);
	    return;
	}
	data.rx_write = w;
    }

    uint8_t w, r;
    w = data.tx_write;
    r = data.tx_read;
    if (r == w) {
	i2c::irq_en(false);
	return;
    }
    uint8_t command = data.tx_buffer[r];
    r = (r + 1) & buffer_mask;

    switch (command) {
    case cmd_start:
	i2c::start(true);
	data.nacked = false;
	break;
    case cmd_write: {
	uint8_t byte = data.tx_buffer[r];
	r = (r + 1) & buffer_mask;
	if (data.nacked) {
	    data.error_condition = i2c_write_nack;
	    return;
	}
	i2c::tx_byte(byte, true);
	break; }
    case cmd_read:
	if (data.nacked) {
	    data.error_condition = i2c_read_nack;
	    return;
	}
	i2c::prepare_rx_byte(true, true);
	data.reading = true;
	break;
    case cmd_last_read:
	if (data.nacked) {
	    data.error_condition = i2c_read_nack;
	    return;
	}
	i2c::prepare_rx_byte(false, true);
	data.reading = true;
	break;
    case cmd_stop:
	i2c::stop(true);
	data.ready_ = true;
	break;
    };

    data.tx_read = r;
}
