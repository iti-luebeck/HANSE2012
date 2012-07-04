#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include "io_config.h"
#include "gpio.h"
#include "serial.h"
#include "i2ccomm.h"
#include "timer1.h"
#include "checksum.h"
#include "error.h"
#include "packet.h"
#include "i2creg.h"


static uint8_t control_acceleration;
static uint8_t control_left;
static uint8_t control_right;
static uint8_t control_front;
static uint8_t control_back;
static bool recover_mode;

static uint8_t report_a;
static uint8_t report_b;


void resetup(bool initial = false)
{
    cli();
    led = true;
    error_condition = no_error;
    serial::setup();
    i2ccomm::setup();
    control_acceleration = recover_acceleration;
    control_left = 0;
    control_right = 0;
    if (!initial) {
	watchdog::setup(false);
    }
    if (initial) {
	if (watchdog::was_intended_reset()) {
	    control_front = 0;
	    control_back = 0;
	    recover_mode = false;
	} else {
	    control_front = recover_speed;
	    control_back = recover_speed;
	    recover_mode = true;
	    report_a = serial_report_wdt;
	}
    } else {
	control_front = recover_speed;
	control_back = recover_speed;
	recover_mode = true;
    };
    sei();
    _delay_ms(300);
    led = false;
}

void setup()
{
    watchdog::setup(true);
    cli();
    led.set_direction(gpio::output);
    report_a = 0;
    report_b = 0;
    for (int i = 0; i < 4; i++) {
	_delay_ms(100);
	led = true;
	_delay_ms(100);
	led = false;
	wdt_reset();
    }
    set_sleep_mode(SLEEP_MODE_IDLE);
    resetup(true);
}

bool serial_receive_message()
{
    packet_reader p;
    p.wait_for_packet();
    uint8_t length = p.read_byte();
    uint8_t cmd = p.read_byte();
    uint8_t a, l, r, f, b;
    bool rec;
    switch (cmd) {
    case serial_cmd_set_speed:
	if (length != 6)
	    return false;
	a = p.read_byte();
	l = p.read_byte();
	r = p.read_byte();
	f = p.read_byte();
	b = p.read_byte();
	rec = true;
	break;
    case serial_cmd_stop:
	if (length != 1)
	    return false;
	a = recover_acceleration;
	l = 0;
	r = 0;
	f = 0;
	b = 0;
	rec = false;
	break;
    default:
	return false;
    }
    if (p.check_packet()) {
	control_acceleration = a;
	control_left  = l;
	control_right = r;
	control_front = f;
	control_back  = b;
	recover_mode = rec;
	return true;
    } else {
	return false;
    }
}

enum pressure_registers {
    register_calib = 0,
    register_pressure_raw = 8,
    register_temp_raw = 10,
    register_pressure = 12,
    register_temp = 14,
    register_status = 17,
    register_counter = 20
};

bool update_motor_speeds()
{
    // mode
    i2creg::write_reg8(motor_right_front_addr,   0, 1);
    i2creg::write_reg8(motor_left_back_addr,   0, 1);
    // acceleration
    i2creg::write_reg8(motor_right_front_addr,   3, control_acceleration);
    i2creg::write_reg8(motor_left_back_addr,     3, control_acceleration);
    // speeds
    i2creg::write_reg8(motor_right_front_addr,   1, control_right);
    i2creg::write_reg8(motor_right_front_addr,   2, control_front);
    i2creg::write_reg8(motor_left_back_addr,     1, control_left);
    i2creg::write_reg8(motor_left_back_addr,     2, control_back);
    return !error_condition;
}

bool read_and_send_pressure_temp()
{
    packet_writer w;

    int16_t pressure = i2creg::read_reg16(pressure_addr, register_pressure);
    int16_t temp = i2creg::read_reg16(pressure_addr, register_temp);

    w.begin_packet();
    w.write_byte(5);
    w.write_byte(serial_cmd_temp_pressure);
    w.write_byte(pressure & 0xFF);
    w.write_byte(pressure >> 8);
    w.write_byte(temp & 0xFF);
    w.write_byte(temp >> 8);
    w.end_packet();

    return !error_condition;
}

bool loop()
{
    while (!serial_receive_message()) {
	if (error_condition == checksum_error) {
	    error_condition = no_error;
	} else {
	    return false;
	}
    }

    if(!update_motor_speeds())
	return false;
    if(!read_and_send_pressure_temp())
	return false;

    watchdog::progress();

    return !error_condition;
}

bool recover()
{
    if (error_condition == no_error)
	error_condition = serial::error();
    if (error_condition != serial_abort)
	return false;
    error_condition = no_error;
    serial::finish();
    serial::setup();
    control_acceleration = recover_acceleration;
    control_left = 0;
    control_right = 0;
    if (recover_mode) {
	control_front = recover_speed;
	control_back = recover_speed;
    } else {
	control_front = 0;
	control_back = 0;
    }
    if (!update_motor_speeds())
	return false;
    watchdog::progress();
    return true;
}

void report_prepare()
{
    if (error_condition == no_error)
	error_condition = serial::error();
    if (error_condition == no_error)
	error_condition = i2ccomm::error();
    if (error_condition == no_error)
	error_condition = unknown_error;
    report_a = serial_report_err;
    report_b = error_condition;
}

void report()
{
    if (!report_a)
	return;
    for (uint8_t i = 0; i < 20; i++)
	serial::write_byte(0);
    packet_writer p;
    p.begin_packet();
    p.write_byte(3);
    p.write_byte(serial_cmd_report);
    p.write_byte(report_a);
    p.write_byte(report_b);
    p.end_packet();
    report_a = 0;
}

int main()
{
    setup();
    while(true) {
	report();
	update_motor_speeds();
	while(true) {
	    if (!loop()) {
		if (!recover()) {
		    report_prepare();
		    break;
		}
	    }
	}
	resetup();
    }
}
