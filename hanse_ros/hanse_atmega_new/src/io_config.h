#ifndef IO_CONFIG_H
#define IO_CONFIG_H

#include "gpio.h"
#include "usart.h"

static constexpr gpio::pin led = gpio::portD(7, true);

static constexpr unsigned long baudrate = 57600;

static constexpr usart::port serial_port = usart::port0;
#define SERIAL_IRQ_PREFIX USART0

static constexpr unsigned long i2c_freq = 40000;

// we use 8 bit/even addresses
static constexpr uint8_t pressure_addr = 0x50;
static constexpr uint8_t motor_left_back_addr = 0xBE;
static constexpr uint8_t motor_right_front_addr = 0xB0;


// sync markers of the serial protocol
static constexpr uint8_t serial_magic_a = 0x13;
static constexpr uint8_t serial_magic_b = 0x37;

// pc -> atmega
static constexpr uint8_t serial_cmd_set_speed = 0x01;
static constexpr uint8_t serial_cmd_stop = 0x02;
// atmega -> pc
static constexpr uint8_t serial_cmd_temp_pressure = 0x03;
static constexpr uint8_t serial_cmd_report = 0x04;

static constexpr uint8_t serial_report_wdt = 0x01;
static constexpr uint8_t serial_report_err = 0x02;


// recover mode
static constexpr int8_t recover_speed = 50;
static constexpr uint8_t recover_acceleration = 10;

// watchdog settings

static constexpr uint16_t io_watchdog_timer = 0x700;
static constexpr uint8_t io_watchdog_serial_limit = 5;
static constexpr uint8_t io_watchdog_i2c_limit = 10; // > serial_limit
#endif
