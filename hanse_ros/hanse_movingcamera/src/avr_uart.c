#include <avr/io.h>

#define BAUD 57600
#define UBRR_VAL ((F_CPU / (16UL * BAUD)) - 1)


// Initialize the UART
void avr_uart_init(void)
{
  // Set the Baud rate
  UBRR0H = (UBRR_VAL >> 8);
  UBRR0L = UBRR_VAL;
  // Enable bidirectional UART
  UCSR0B |= _BV(RXEN0) | _BV(TXEN0);
  // Use 8-bit characters
  UCSR0C |= _BV(UCSZ10) | _BV(UCSZ11);
  
}


// Send one char (blocking)
void avr_uart_send_byte(uint8_t tx_byte)
{
  // Wait to be able to transmit
  while((UCSR0A & _BV(UDRE0)) == 0)
    asm volatile("nop"::);
  // Put the data into the send buffer
  UDR0 = tx_byte;
}


// Get one char if available, otherwise -1
int16_t avr_uart_receive_byte(void)
{
  if((UCSR0A & _BV(RXC0)) != 0)
  {
    return UDR0;
  }
  else
  {
    return -1;
  }
}
