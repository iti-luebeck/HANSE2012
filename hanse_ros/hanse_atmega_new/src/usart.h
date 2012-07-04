#ifndef USART_H
#define USART_H

#include <avr/io.h>

namespace usart {

    class port;
    class settings;

    class port {
	volatile uint8_t * const udr;
	volatile uint8_t * const ucsrA;
	volatile uint8_t * const ucsrB;
	volatile uint8_t * const ucsrC;
	volatile uint16_t * const ubrr;
    public:
	constexpr port(volatile uint8_t &udr,
		       volatile uint8_t &ucsrA,
		       volatile uint8_t &ucsrB,
		       volatile uint8_t &ucsrC,
		       volatile uint16_t &ubrr) :
	    udr(&udr),
	    ucsrA(&ucsrA),
	    ucsrB(&ucsrB),
	    ucsrC(&ucsrC),
	    ubrr(&ubrr)
	{ }

	inline void set_settings(settings settings) const;
	bool tx_ready() const { return *ucsrA & (1 << UDRE0); };
	void tx_byte(uint8_t byte) const { *udr = byte; };
	uint8_t rx_byte() const { return *udr; }
	bool error() const {
	    return *ucsrA & ((1 << FE0) | (1 << DOR0) | (1 << UPE0));
	}
	void rx_irq_en(bool enable) const {
	    if (enable)
		*ucsrB |= (1 << RXCIE0);
	    else
		*ucsrB &= ~(1 << RXCIE0);
	}
	void tx_irq_en(bool enable) const {
	    if (enable)
		*ucsrB |= (1 << TXCIE0);
	    else
		*ucsrB &= ~(1 << TXCIE0);
	}
	void tx_ready_irq_en(bool enable) const {
	    if (enable)
		*ucsrB |= (1 << UDRIE0);
	    else
		*ucsrB &= ~(1 << UDRIE0);
	}
	void rx_en(bool enable) const {
	    if (enable)
		*ucsrB |= (1 << RXEN0);
	    else
		*ucsrB &= ~(1 << RXEN0);
	}
	void tx_en(bool enable) const {
	    if (enable)
		*ucsrB |= (1 << TXEN0);
	    else
		*ucsrB &= ~(1 << TXEN0);
	}
    };

    class settings {
	friend class port;
	const uint8_t ucsrA;
	const uint8_t ucsrB;
	const uint8_t ucsrC;
	const uint16_t ubrr;
    public:
	constexpr settings(unsigned long baudrate) :
	    ucsrA(0),
	    ucsrB((0 << UCSZ02)),
	    ucsrC((1 << UCSZ01) | (1 << UCSZ00) | (1 << USBS0)),
	    ubrr((F_CPU) / (16 * baudrate) - 1)
	{ }
    };

    inline void port::set_settings(settings settings) const  {
	*ucsrA = settings.ucsrA;
	*ucsrB = settings.ucsrB;
	*ucsrC = settings.ucsrC;
	*ubrr = settings.ubrr;
    }

#ifdef UCSR0A
    static constexpr port port0(UDR0, UCSR0A, UCSR0B, UCSR0C, UBRR0);
#endif

#ifdef UCSR1A
    static constexpr port port1(UDR1, UCSR1A, UCSR1B, UCSR1C, UBRR1);
#endif
};

#endif
