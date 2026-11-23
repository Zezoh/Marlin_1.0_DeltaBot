/*
  MarlinSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 28 September 2010 by Mark Sproul
*/

#ifndef MarlinSerial_h
#define MarlinSerial_h

#include "Marlin.h"

#if !defined(SERIAL_PORT)
#define SERIAL_PORT 0
#endif

// Names of the registers of the selected UART of the ATmega1280/2560
#define SERIAL_REGNAME(registerbase, number, suffix) SERIAL_REGNAME_INTERNAL(registerbase, number, suffix)
#define SERIAL_REGNAME_INTERNAL(registerbase, number, suffix) registerbase##number##suffix

#define M_UCSRxA SERIAL_REGNAME(UCSR, SERIAL_PORT, A)
#define M_UCSRxB SERIAL_REGNAME(UCSR, SERIAL_PORT, B)
#define M_RXENx SERIAL_REGNAME(RXEN, SERIAL_PORT, )
#define M_TXENx SERIAL_REGNAME(TXEN, SERIAL_PORT, )
#define M_RXCIEx SERIAL_REGNAME(RXCIE, SERIAL_PORT, )
#define M_UDREx SERIAL_REGNAME(UDRE, SERIAL_PORT, )
#define M_UDRx SERIAL_REGNAME(UDR, SERIAL_PORT, )
#define M_UBRRxH SERIAL_REGNAME(UBRR, SERIAL_PORT, H)
#define M_UBRRxL SERIAL_REGNAME(UBRR, SERIAL_PORT, L)
#define M_RXCx SERIAL_REGNAME(RXC, SERIAL_PORT, )
#define M_USARTx_RX_vect SERIAL_REGNAME(USART, SERIAL_PORT, _RX_vect)
#define M_U2Xx SERIAL_REGNAME(U2X, SERIAL_PORT, )

// Ring buffer for the received characters: rx_buffer.head is the index where the next received
// character is written, rx_buffer.tail the index of the next character to read.
// The size must be a power of 2 below 256: the indexes are 8 bit (atomic and fast) and wrap with a mask.
#define RX_BUFFER_SIZE 128

struct ring_buffer {
  unsigned char buffer[RX_BUFFER_SIZE];
  volatile uint8_t head;
  volatile uint8_t tail;
};

extern ring_buffer rx_buffer;

// Store a received character, unless the buffer is full (the character is then lost)
FORCE_INLINE void store_char(unsigned char c)
{
  uint8_t i = (rx_buffer.head + 1) & (RX_BUFFER_SIZE - 1);
  if (i != rx_buffer.tail) {
    rx_buffer.buffer[rx_buffer.head] = c;
    rx_buffer.head = i;
  }
}

class MarlinSerial
{
public:
  void begin(long baud);
  int read(void);
  void flush(void);  // discards the received characters

  FORCE_INLINE int available(void)
  {
    return (uint8_t)(rx_buffer.head - rx_buffer.tail) & (RX_BUFFER_SIZE - 1);
  }

  // Transmit is polled: wait for the data register to be empty, then send
  FORCE_INLINE void write(uint8_t c)
  {
    while (!((M_UCSRxA) & (1 << M_UDREx)));
    M_UDRx = c;
  }

  // Move a received character to the buffer without waiting for the interrupt.
  // Called from the stepper interrupt, which blocks the serial interrupt.
  FORCE_INLINE void checkRx(void)
  {
    if ((M_UCSRxA & (1 << M_RXCx)) != 0) {
      store_char(M_UDRx);
    }
  }

  FORCE_INLINE void write(const char *str)
  {
    while (*str)
      write(*str++);
  }

  FORCE_INLINE void print(const char *str) { write(str); }
  void print(char, int = 0);
  void print(unsigned char, int = 0);
  void print(int, int = 10);
  void print(unsigned int, int = 10);
  void print(long, int = 10);
  void print(unsigned long, int = 10);
  void print(double, int = 2);

private:
  void printNumber(unsigned long, uint8_t);
  void printFloat(double, uint8_t);
};

extern MarlinSerial MSerial;

#endif
