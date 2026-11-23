// Fast pin access for the pins the firmware uses (ATmega1280 / ATmega2560): READ, WRITE, SET_INPUT, SET_OUTPUT take the
// Arduino pin number. Add a DIO line for a new pin: DIO<n>_PIN is its bit, _RPORT the PINx, _WPORT the PORTx, _DDR the DDRx register.

#ifndef _FASTIO_ARDUINO_H
#define _FASTIO_ARDUINO_H

#include <avr/io.h>

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error The pin table below is for the ATmega1280 and ATmega2560 only
#endif

#ifndef MASK
#define MASK(PIN)  (1 << PIN)
#endif

#define _READ(IO) ((bool)(DIO ## IO ## _RPORT & MASK(DIO ## IO ## _PIN)))

// Ports above 0x100 (H, J, K, L) need a critical section: the write is not one instruction
#define _WRITE_NC(IO, v)  do { if (v) {DIO ##  IO ## _WPORT |= MASK(DIO ## IO ## _PIN); } else {DIO ##  IO ## _WPORT &= ~MASK(DIO ## IO ## _PIN); }; } while (0)
#define _WRITE_C(IO, v)   do { CRITICAL_SECTION_START; _WRITE_NC(IO, v); CRITICAL_SECTION_END; } while (0)
#define _WRITE(IO, v)  do {  if (&(DIO ##  IO ## _RPORT) >= (uint8_t *)0x100) {_WRITE_C(IO, v); } else {_WRITE_NC(IO, v); }; } while (0)

#define _SET_INPUT(IO) do {DIO ##  IO ## _DDR &= ~MASK(DIO ## IO ## _PIN); } while (0)
#define _SET_OUTPUT(IO) do {DIO ##  IO ## _DDR |=  MASK(DIO ## IO ## _PIN); } while (0)

// why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
#define READ(IO)  _READ(IO)
#define WRITE(IO, v)  _WRITE(IO, v)
#define SET_INPUT(IO)  _SET_INPUT(IO)
#define SET_OUTPUT(IO)  _SET_OUTPUT(IO)

#define DIO2_PIN PINE4
#define DIO2_RPORT PINE
#define DIO2_WPORT PORTE
#define DIO2_DDR DDRE
#define DIO3_PIN PINE5
#define DIO3_RPORT PINE
#define DIO3_WPORT PORTE
#define DIO3_DDR DDRE
#define DIO9_PIN PINH6
#define DIO9_RPORT PINH
#define DIO9_WPORT PORTH
#define DIO9_DDR DDRH
#define DIO10_PIN PINB4
#define DIO10_RPORT PINB
#define DIO10_WPORT PORTB
#define DIO10_DDR DDRB
#define DIO12_PIN PINB6
#define DIO12_RPORT PINB
#define DIO12_WPORT PORTB
#define DIO12_DDR DDRB
#define DIO14_PIN PINJ1
#define DIO14_RPORT PINJ
#define DIO14_WPORT PORTJ
#define DIO14_DDR DDRJ
#define DIO15_PIN PINJ0
#define DIO15_RPORT PINJ
#define DIO15_WPORT PORTJ
#define DIO15_DDR DDRJ
#define DIO19_PIN PIND2
#define DIO19_RPORT PIND
#define DIO19_WPORT PORTD
#define DIO19_DDR DDRD
#define DIO24_PIN PINA2
#define DIO24_RPORT PINA
#define DIO24_WPORT PORTA
#define DIO24_DDR DDRA
#define DIO26_PIN PINA4
#define DIO26_RPORT PINA
#define DIO26_WPORT PORTA
#define DIO26_DDR DDRA
#define DIO28_PIN PINA6
#define DIO28_RPORT PINA
#define DIO28_WPORT PORTA
#define DIO28_DDR DDRA
#define DIO38_PIN PIND7
#define DIO38_RPORT PIND
#define DIO38_WPORT PORTD
#define DIO38_DDR DDRD
#define DIO46_PIN PINL3
#define DIO46_RPORT PINL
#define DIO46_WPORT PORTL
#define DIO46_DDR DDRL
#define DIO48_PIN PINL1
#define DIO48_RPORT PINL
#define DIO48_WPORT PORTL
#define DIO48_DDR DDRL
#define DIO50_PIN PINB3
#define DIO50_RPORT PINB
#define DIO50_WPORT PORTB
#define DIO50_DDR DDRB
#define DIO51_PIN PINB2
#define DIO51_RPORT PINB
#define DIO51_WPORT PORTB
#define DIO51_DDR DDRB
#define DIO52_PIN PINB1
#define DIO52_RPORT PINB
#define DIO52_WPORT PORTB
#define DIO52_DDR DDRB
#define DIO53_PIN PINB0
#define DIO53_RPORT PINB
#define DIO53_WPORT PORTB
#define DIO53_DDR DDRB
#define DIO54_PIN PINF0
#define DIO54_RPORT PINF
#define DIO54_WPORT PORTF
#define DIO54_DDR DDRF
#define DIO55_PIN PINF1
#define DIO55_RPORT PINF
#define DIO55_WPORT PORTF
#define DIO55_DDR DDRF
#define DIO56_PIN PINF2
#define DIO56_RPORT PINF
#define DIO56_WPORT PORTF
#define DIO56_DDR DDRF
#define DIO60_PIN PINF6
#define DIO60_RPORT PINF
#define DIO60_WPORT PORTF
#define DIO60_DDR DDRF
#define DIO61_PIN PINF7
#define DIO61_RPORT PINF
#define DIO61_WPORT PORTF
#define DIO61_DDR DDRF
#define DIO62_PIN PINK0
#define DIO62_RPORT PINK
#define DIO62_WPORT PORTK
#define DIO62_DDR DDRK

#endif /* _FASTIO_ARDUINO_H */
