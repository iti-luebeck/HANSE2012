#ifndef TIMER1_H
#define TIMER1_H

#include <avr/io.h>

// TODO: this class is not a generic timer class
// but only for hanse_atmega_new
class timer1 {
public:
    static void setup()
    {
	TCCR1A = 0;
	TCCR1B = (1<<CS12) | (1<<CS10); // clk / 1024
	TCCR1C = 0;
	TCNT1 = 0;
	TIMSK1 = 0;
	TIFR1 = (1<<TOV1);
    }
    static void reset_timer(uint16_t value = 0)
    {
	TCNT1 = value;
    }
    static void overflow_irq_en(bool enabled = true)
    {
	if (enabled)
	    TIMSK1 |= (1<<TOIE1);
	else
	    TIMSK1 &= ~(1<<TOIE1);
    }
};

#endif
