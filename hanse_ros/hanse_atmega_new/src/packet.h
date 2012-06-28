#ifndef PACKET_H
#define PACKET_H

#include "serial.h"
#include "checksum.h"
#include "error.h"
#include "watchdog.h"

class packet_reader {
    checksum ck;
public:
    void wait_for_packet()
    {
	uint8_t b = serial::read_byte();
	while (true) {
	    if (serial::error())
		return;
	    if (b == serial_magic_a) {
		b = serial::read_byte();
		if (b == serial_magic_b) {
		    break;
		}
	    } else {
		b = serial::read_byte();
	    }
	}
    }

    uint8_t read_byte()
    {
	return ck.add_byte(serial::read_byte());
    }

    bool check_packet()
    {
	if (serial::read_byte() != ck.checksum_a()) {
	    error_condition = checksum_error;
	} else if (serial::read_byte() != ck.checksum_b()) {
	    error_condition = checksum_error;
	}
	if (serial::error()) {
	    error_condition = serial::error();
	}
	ck.reset();
	if (!error_condition) {
	    watchdog::serial_progress();
	    return true;
	} else {
	    return false;
	}
    }
};

class packet_writer {
    checksum ck;
public:
    void begin_packet()
    {
	serial::write_byte(serial_magic_a);
	serial::write_byte(serial_magic_b);
    }
    void write_byte(uint8_t byte)
    {
	serial::write_byte(ck.add_byte(byte));
    }
    void end_packet()
    {
	serial::write_byte(ck.checksum_a());
	serial::write_byte(ck.checksum_b());
    }
};

#endif
