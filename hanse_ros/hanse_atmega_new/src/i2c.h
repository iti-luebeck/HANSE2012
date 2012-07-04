#ifndef I2C_H
#define I2C_H

#include <util/twi.h>

class i2c {
public:
    static void setup(unsigned long freq) {
	TWBR = ((F_CPU / freq) - 16) / 2;
	TWSR = 0;
	TWCR = 0;
    }
    static void enable(bool enabled = true) {
	if (enabled)
	    TWCR |= (1<<TWEN);
	else
	    TWCR &= ~(1<<TWEN);
    }
    static bool ready() {
	return (TWCR & (1<<TWINT));
    }
    static void start(bool irq_en = false) {
	TWCR = ((irq_en ? 1 : 0)<<TWIE) |
	    (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    }
    static void stop(bool irq_en = false) {
	TWCR = ((irq_en ? 1 : 0)<<TWIE) |
	    (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
    }
    static void tx_byte(uint8_t byte, bool irq_en = false) {
	TWDR = byte;
	TWCR = ((irq_en ? 1 : 0)<<TWIE) |
	    (1<<TWINT) | (1<<TWEN);
    }
    static void prepare_rx_byte(bool ack = true, bool irq_en = false) {
	TWCR = ((irq_en ? 1 : 0)<<TWIE) |
	    ((ack ? 1 : 0) << TWEA) |
	    (1<<TWINT) | (1<<TWEN);
    }
    static uint8_t rx_byte() {
	return TWDR;
    }
    static uint8_t status() {
	return TW_STATUS;
    }
    static void irq_en(bool irq_en = true) {
	// avoid acknowledging of the irq
	if (irq_en)
	    TWCR = (TWCR | (1<<TWIE)) & ~(1<<TWINT);
	else
	    TWCR &= ~((1<<TWIE) | (1<<TWINT));
    }
};

#endif
