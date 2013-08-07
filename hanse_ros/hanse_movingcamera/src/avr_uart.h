#ifndef _AVR_UART_H_
#define _AVR_UART_H_

#include <stdint.h>
#include <avr/io.h>

void avr_uart_init(void);
void avr_uart_send_byte(uint8_t tx_byte);
int8_t avr_uart_receive_byte(void);
int8_t avr_uart_receive_servo_position(int8_t servo_position_old);
void avr_uart_send_position(int8_t servo_position_degree);

#endif
