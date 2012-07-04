#include "serial.h"

#include <avr/interrupt.h>

#define ISR2(A, B) ISR2_(A, B)
#define ISR2_(A, B) ISR(A ## B)

serial serial::data;

ISR2(SERIAL_IRQ_PREFIX, _RX_vect) { serial::rx_irq(); }
ISR2(SERIAL_IRQ_PREFIX, _UDRE_vect) { serial::tx_irq(); }

void serial::rx_irq() {
    if (serial_port.error()) {
	data.error_condition = serial_receive_error;
	return;
    }
    uint8_t w, r;
    w = data.rx_write;
    r = data.rx_read;
    data.rx_buffer[w] = serial_port.rx_byte();
    w = (w + 1) & buffer_mask;
    if (w == r) {
	data.error_condition = serial_receive_overflow;
	return;
    }
    data.rx_write = w;
}

void serial::tx_irq() {
    uint8_t w, r;
    w = data.tx_write;
    r = data.tx_read;
    if (r == w) {
	serial_port.tx_ready_irq_en(false);
	return;
    }
    serial_port.tx_byte(data.tx_buffer[r]);
    data.tx_read = (r + 1) & buffer_mask;
}
