/*
  MarlinSerial.cpp - Hardware serial library for Wiring
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

  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
*/

#include "Marlin.h"
#include "MarlinSerial.h"

ring_buffer rx_buffer = { { 0 }, 0, 0 };

SIGNAL(M_USARTx_RX_vect)
{
  store_char(M_UDRx);
}

void MarlinSerial::begin(long baud)
{
  uint16_t baud_setting;
  bool useU2X = true;

#if F_CPU == 16000000UL && SERIAL_PORT == 0
  // hard-coded exception for compatibility with the bootloader shipped
  // with the Duemilanove and previous boards and the firmware on the 8U2
  // on the Uno and Mega 2560.
  if (baud == 57600) {
    useU2X = false;
  }
#endif

  if (useU2X) {
    M_UCSRxA = 1 << M_U2Xx;
    baud_setting = (F_CPU / 4 / baud - 1) / 2;
  } else {
    M_UCSRxA = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }

  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
  M_UBRRxH = baud_setting >> 8;
  M_UBRRxL = baud_setting;

  sbi(M_UCSRxB, M_RXENx);
  sbi(M_UCSRxB, M_TXENx);
  sbi(M_UCSRxB, M_RXCIEx);
}

int MarlinSerial::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (rx_buffer.head == rx_buffer.tail) {
    return -1;
  }
  unsigned char c = rx_buffer.buffer[rx_buffer.tail];
  rx_buffer.tail = (rx_buffer.tail + 1) & (RX_BUFFER_SIZE - 1);
  return c;
}

void MarlinSerial::flush()
{
  // Discard everything received: moving the head to the tail instead of the tail to the head
  // is safe if the receive interrupt occurs in between; the opposite order could make the
  // buffer look full.
  rx_buffer.head = rx_buffer.tail;
}

//////////////////////////////// Printing /////////////////////////////////////

void MarlinSerial::print(char c, int base)
{
  print((long) c, base);
}

void MarlinSerial::print(unsigned char b, int base)
{
  print((unsigned long) b, base);
}

void MarlinSerial::print(int n, int base)
{
  print((long) n, base);
}

void MarlinSerial::print(unsigned int n, int base)
{
  print((unsigned long) n, base);
}

void MarlinSerial::print(long n, int base)
{
  if (base == 0) {
    write(n);
  } else if (base == 10) {
    if (n < 0) {
      write('-');
      printNumber(-(unsigned long)n, 10);  // also right for the most negative number
    } else {
      printNumber(n, 10);
    }
  } else {
    printNumber(n, base);
  }
}

void MarlinSerial::print(unsigned long n, int base)
{
  if (base == 0) write(n);
  else printNumber(n, base);
}

void MarlinSerial::print(double n, int digits)
{
  printFloat(n, digits);
}

void MarlinSerial::printNumber(unsigned long n, uint8_t base)
{
  unsigned char buf[8 * sizeof(long)];
  uint8_t i = 0;

  if (n == 0) {
    write('0');
    return;
  }

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    write((char) (buf[i - 1] < 10 ? '0' + buf[i - 1] : 'A' + buf[i - 1] - 10));
}

void MarlinSerial::printFloat(double number, uint8_t digits)
{
  // Not a number or infinite: the integer conversion below would be undefined
  if (isnan(number)) { write("nan"); return; }
  if (isinf(number)) { write(number < 0 ? "-inf" : "inf"); return; }

  // Handle negative numbers
  if (number < 0.0)
  {
    write('-');
    number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  printNumber(int_part, 10);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    write('.');

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    write('0' + toPrint);
    remainder -= toPrint;
  }
}

MarlinSerial MSerial;
