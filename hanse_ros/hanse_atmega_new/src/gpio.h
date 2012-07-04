#ifndef GPIO_H
#define GPIO_H

#include <inttypes.h>
#include <avr/io.h>

namespace gpio {

    class port;
    class pin;

    enum direction : uint8_t {
	output,
	input
    };

    /*    constexpr direction output = direction::output;
    constexpr direction input = direction::input;
    */
    class port {
	friend class pin;
	volatile uint8_t * const ddr_;
	volatile uint8_t * const port_;
	volatile uint8_t * const pin_;
    public:
	constexpr port(volatile uint8_t &ddr,
			    volatile uint8_t &port,
			    volatile uint8_t &pin) :
	    ddr_(&ddr),
	    port_(&port),
	    pin_(&pin)
	{ }
	inline constexpr pin operator()(uint8_t bit,
					bool inverted_in = false,
					bool inverted_out = false);
    };

    class pin {
	const port port_;
	const uint8_t mask;
	const bool inverted_out;
	const bool inverted_in;
    public:
	constexpr pin(port port, uint8_t bit,
		      bool inverted_out = false,
		      bool inverted_in = false) :
	    port_(port),
	    mask(1<<bit),
	    inverted_out(inverted_out),
	    inverted_in(inverted_in)
	{
	}

	operator bool() const {
	    return bool((*port_.pin_) & mask) != inverted_in;
	}

	bool operator =(bool value) const {
	    if (value != inverted_out)
		*port_.port_ |= mask;
	    else
		*port_.port_ &= ~mask;
	    return value;
	}

	void set_direction(gpio::direction direction) const {
	    if (direction == gpio::output)
		*port_.ddr_ |= mask;
	    else
		*port_.ddr_ &= ~mask;
	}
    };


    inline constexpr pin port::operator()(uint8_t bit,
					  bool inverted_in,
					  bool inverted_out)
    {
	return pin(*this, bit, inverted_in, inverted_out);
    }

#ifdef PORTA
    static constexpr port portA = port(DDRA, PORTA, PINA);
#endif
#ifdef PORTB
    static constexpr port portB = port(DDRB, PORTB, PINB);
#endif
#ifdef PORTC
    static constexpr port portC = port(DDRC, PORTC, PINC);
#endif
#ifdef PORTD
    static constexpr port portD = port(DDRD, PORTD, PIND);
#endif
}
#endif
