/**
 * Implementation details for displayWrite(...) function
 *
 * Sequentially sets and displays content on a set of eight 7-segment displays,
 * multiplexing with fast period such that content looks static due to
 * persistence of vision.
 *
 * Timing is controlled through Timer1 and its associated ISR
 *
 * Content is shifted out through a pair of 74HC595 shift registers
 * See schematic layout for wiring details
 *
 */
#include "DisplayWrite.h"
#include <Arduino.h>
#include <stdint.h>

/*
 * Faster digital write using AVR commands
 *
 * Optimized digital functions for AVR microcontrollers
 * by Watterott electronic (www.watterott.com)
 * based on http://code.google.com/p/digitalwritefast
 */

#ifndef __digitalWriteFast_h_
#define __digitalWriteFast_h_ 1

/* general macros/defines */
#ifndef BIT_READ
# define BIT_READ(value, bit)            ((value) &   (1UL << (bit)))
#endif
#ifndef BIT_SET
# define BIT_SET(value, bit)             ((value) |=  (1UL << (bit)))
#endif
#ifndef BIT_CLEAR
# define BIT_CLEAR(value, bit)           ((value) &= ~(1UL << (bit)))
#endif
#ifndef BIT_WRITE
# define BIT_WRITE(value, bit, bitvalue) (bitvalue ? BIT_SET(value, bit) : BIT_CLEAR(value, bit))
#endif

#ifndef SWAP
#define SWAP(x,y) do{ (x)=(x)^(y); (y)=(x)^(y); (x)=(x)^(y); }while(0)
#endif

#ifndef DEC
# define DEC (10)
#endif
#ifndef HEX
# define HEX (16)
#endif
#ifndef OCT
# define OCT (8)
#endif
#ifndef BIN
# define BIN (2)
#endif


/* workarounds for ARM microcontrollers */
#if (!defined(__AVR__) || defined(ARDUINO_ARCH_SAM))
#ifndef PROGMEM
# define PROGMEM
#endif
#ifndef PGM_P
# define PGM_P const char *
#endif
#ifndef PSTR
# define PSTR(str) (str)
#endif

#ifndef memcpy_P
# define memcpy_P(dest, src, num) memcpy((dest), (src), (num))
#endif
#ifndef strcpy_P
# define strcpy_P(dst, src)       strcpy((dst), (src))
#endif
#ifndef strcat_P
# define strcat_P(dst, src)       strcat((dst), (src))
#endif
#ifndef strcmp_P
# define strcmp_P(a, b)           strcmp((a), (b))
#endif
#ifndef strcasecmp_P
# define strcasecmp_P(a, b)       strcasecmp((a), (b))
#endif
#ifndef strncmp_P
# define strncmp_P(a, b, n)       strncmp((a), (b), (n))
#endif
#ifndef strncasecmp_P
# define strncasecmp_P(a, b, n)   strncasecmp((a), (b), (n))
#endif
#ifndef strstr_P
# define strstr_P(a, b)           strstr((a), (b))
#endif
#ifndef strlen_P
# define strlen_P(a)              strlen((a))
#endif
#ifndef sprintf_P
# define sprintf_P(s, f, ...)     sprintf((s), (f), __VA_ARGS__)
#endif

#ifndef pgm_read_byte
# define pgm_read_byte(addr)      (*(const unsigned char *)(addr))
#endif
#ifndef pgm_read_word
# define pgm_read_word(addr)      (*(const unsigned short *)(addr))
#endif
#ifndef pgm_read_dword
# define pgm_read_dword(addr)     (*(const unsigned long *)(addr))
#endif

#endif


/* digital functions */
/*#ifndef digitalPinToPortReg */
#define SPI_SW_SS_PIN   (10) /* SS on Uno (for software SPI)   */
#define SPI_SW_MOSI_PIN (11) /* MOSI on Uno (for software SPI) */
#define SPI_SW_MISO_PIN (12) /* MISO on Uno (for software SPI) */
#define SPI_SW_SCK_PIN  (13) /* SCK on Uno (for software SPI) */


/* --- Arduino Due --- */
#if (defined(ARDUINO_SAM_DUE) || defined(__SAM3X8E__))

#define UART_RX_PIN     (0)
#define UART_TX_PIN     (1)

#define I2C_SDA_PIN     (20)
#define I2C_SCL_PIN     (21)

#define SPI_HW_SS_PIN   (78) /* SS0:77, SS1:87, SS2:86, SS3:78 */
#define SPI_HW_MOSI_PIN (75) /* 75 */
#define SPI_HW_MISO_PIN (74) /* 74 */
#define SPI_HW_SCK_PIN  (76) /* 76 */


/* --- Arduino Zero --- */
#elif (defined(ARDUINO_SAM_ZERO) || defined(__SAMD21G18A__))

#define UART_RX_PIN     (0)
#define UART_TX_PIN     (1)

#define I2C_SDA_PIN     (16)
#define I2C_SCL_PIN     (17)

#define SPI_HW_SS_PIN   (14) /* 14 */
#define SPI_HW_MOSI_PIN (21) /* 21 */
#define SPI_HW_MISO_PIN (18) /* 18 */
#define SPI_HW_SCK_PIN  (20) /* 20 */


/*  --- Arduino Mega --- */
#elif (defined(ARDUINO_AVR_MEGA) || \
       defined(ARDUINO_AVR_MEGA1280) || \
       defined(ARDUINO_AVR_MEGA2560) || \
       defined(__AVR_ATmega1280__) || \
       defined(__AVR_ATmega1281__) || \
       defined(__AVR_ATmega2560__) || \
       defined(__AVR_ATmega2561__))

#define UART_RX_PIN     (0) /* PE0 */
#define UART_TX_PIN     (1) /* PE1 */

#define I2C_SDA_PIN     (20)
#define I2C_SCL_PIN     (21)

#define SPI_HW_SS_PIN   (53) /* PB0 */
#define SPI_HW_MOSI_PIN (51) /* PB2 */
#define SPI_HW_MISO_PIN (50) /* PB3 */
#define SPI_HW_SCK_PIN  (52) /* PB1 */

#define __digitalPinToPortReg(P) \
(((P) >= 22 && (P) <= 29) ? &PORTA : \
((((P) >= 10 && (P) <= 13) || ((P) >= 50 && (P) <= 53)) ? &PORTB : \
(((P) >= 30 && (P) <= 37) ? &PORTC : \
((((P) >= 18 && (P) <= 21) || (P) == 38) ? &PORTD : \
((((P) >= 0 && (P) <= 3) || (P) == 5) ? &PORTE : \
(((P) >= 54 && (P) <= 61) ? &PORTF : \
((((P) >= 39 && (P) <= 41) || (P) == 4) ? &PORTG : \
((((P) >= 6 && (P) <= 9) || (P) == 16 || (P) == 17) ? &PORTH : \
(((P) == 14 || (P) == 15) ? &PORTJ : \
(((P) >= 62 && (P) <= 69) ? &PORTK : &PORTL))))))))))

#define __digitalPinToDDRReg(P) \
(((P) >= 22 && (P) <= 29) ? &DDRA : \
((((P) >= 10 && (P) <= 13) || ((P) >= 50 && (P) <= 53)) ? &DDRB : \
(((P) >= 30 && (P) <= 37) ? &DDRC : \
((((P) >= 18 && (P) <= 21) || (P) == 38) ? &DDRD : \
((((P) >= 0 && (P) <= 3) || (P) == 5) ? &DDRE : \
(((P) >= 54 && (P) <= 61) ? &DDRF : \
((((P) >= 39 && (P) <= 41) || (P) == 4) ? &DDRG : \
((((P) >= 6 && (P) <= 9) || (P) == 16 || (P) == 17) ? &DDRH : \
(((P) == 14 || (P) == 15) ? &DDRJ : \
(((P) >= 62 && (P) <= 69) ? &DDRK : &DDRL))))))))))

#define __digitalPinToPINReg(P) \
(((P) >= 22 && (P) <= 29) ? &PINA : \
((((P) >= 10 && (P) <= 13) || ((P) >= 50 && (P) <= 53)) ? &PINB : \
(((P) >= 30 && (P) <= 37) ? &PINC : \
((((P) >= 18 && (P) <= 21) || (P) == 38) ? &PIND : \
((((P) >= 0 && (P) <= 3) || (P) == 5) ? &PINE : \
(((P) >= 54 && (P) <= 61) ? &PINF : \
((((P) >= 39 && (P) <= 41) || (P) == 4) ? &PING : \
((((P) >= 6 && (P) <= 9) || (P) == 16 || (P) == 17) ? &PINH : \
(((P) == 14 || (P) == 15) ? &PINJ : \
(((P) >= 62 && (P) <= 69) ? &PINK : &PINL))))))))))

#define __digitalPinToBit(P) \
(((P) >=  7 && (P) <=  9) ? (P) - 3 : \
(((P) >= 10 && (P) <= 13) ? (P) - 6 : \
(((P) >= 22 && (P) <= 29) ? (P) - 22 : \
(((P) >= 30 && (P) <= 37) ? 37 - (P) : \
(((P) >= 39 && (P) <= 41) ? 41 - (P) : \
(((P) >= 42 && (P) <= 49) ? 49 - (P) : \
(((P) >= 50 && (P) <= 53) ? 53 - (P) : \
(((P) >= 54 && (P) <= 61) ? (P) - 54 : \
(((P) >= 62 && (P) <= 69) ? (P) - 62 : \
(((P) == 0 || (P) == 15 || (P) == 17 || (P) == 21) ? 0 : \
(((P) == 1 || (P) == 14 || (P) == 16 || (P) == 20) ? 1 : \
(((P) == 19) ? 2 : \
(((P) == 5 || (P) == 6 || (P) == 18) ? 3 : \
(((P) == 2) ? 4 : \
(((P) == 3 || (P) == 4) ? 5 : 7)))))))))))))))


/*  --- Arduino 644 --- */
#elif (defined(__AVR_ATmega644__) || \
       defined(__AVR_ATmega644P__))

#define UART_RX_PIN     (8) /* PD0 */
#define UART_TX_PIN     (9) /* PD1 */

#define I2C_SDA_PIN     (17) /* PC1 */
#define I2C_SCL_PIN     (16) /* PC0 */

#define SPI_HW_SS_PIN   (4) /* PB4 */
#define SPI_HW_MOSI_PIN (5) /* PB5 */
#define SPI_HW_MISO_PIN (6) /* PB6 */
#define SPI_HW_SCK_PIN  (7) /* PB7 */

#define __digitalPinToPortReg(P) \
(((P) >= 0 && (P) <= 7) ? &PORTB : (((P) >= 8 && (P) <= 15) ? &PORTD : (((P) >= 16 && (P) <= 23) ? &PORTC : &PORTA)))
#define __digitalPinToDDRReg(P) \
(((P) >= 0 && (P) <= 7) ? &DDRB : (((P) >= 8 && (P) <= 15) ? &DDRD : (((P) >= 8 && (P) <= 15) ? &DDRC : &DDRA)))
#define __digitalPinToPINReg(P) \
(((P) >= 0 && (P) <= 7) ? &PINB : (((P) >= 8 && (P) <= 15) ? &PIND : (((P) >= 8 && (P) <= 15) ? &PINC : &PINA)))
#define __digitalPinToBit(P) \
(((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 15) ? (P) - 8 : (((P) >= 16 && (P) <= 23) ? (P) - 16 : (P) - 24)))


/*  --- Arduino Leonardo --- */
#elif (defined(ARDUINO_AVR_LEONARDO) || \
       defined(__AVR_ATmega16U4__) || \
       defined(__AVR_ATmega32U4__))

#define UART_RX_PIN     (0) /* PD2 */
#define UART_TX_PIN     (1) /* PD3 */

#define I2C_SDA_PIN     (2) /* PD1 */
#define I2C_SCL_PIN     (3) /* PD0 */

#define SPI_HW_SS_PIN   (17) /* PB0 */
#define SPI_HW_MOSI_PIN (16) /* PB2 */
#define SPI_HW_MISO_PIN (14) /* PB3 */
#define SPI_HW_SCK_PIN  (15) /* PB1 */

#define __digitalPinToPortReg(P) \
((((P) >= 0 && (P) <= 4) || (P) == 6 || (P) == 12 || (P) == 24 || (P) == 25 || (P) == 29) ? &PORTD : (((P) == 5 || (P) == 13) ? &PORTC : (((P) >= 18 && (P) <= 23)) ? &PORTF : (((P) == 7) ? &PORTE : &PORTB)))
#define __digitalPinToDDRReg(P) \
((((P) >= 0 && (P) <= 4) || (P) == 6 || (P) == 12 || (P) == 24 || (P) == 25 || (P) == 29) ? &DDRD : (((P) == 5 || (P) == 13) ? &DDRC : (((P) >= 18 && (P) <= 23)) ? &DDRF : (((P) == 7) ? &DDRE : &DDRB)))
#define __digitalPinToPINReg(P) \
((((P) >= 0 && (P) <= 4) || (P) == 6 || (P) == 12 || (P) == 24 || (P) == 25 || (P) == 29) ? &PIND : (((P) == 5 || (P) == 13) ? &PINC : (((P) >= 18 && (P) <= 23)) ? &PINF : (((P) == 7) ? &PINE : &PINB)))
#define __digitalPinToBit(P) \
(((P) >= 8 && (P) <= 11) ? (P) - 4 : (((P) >= 18 && (P) <= 21) ? 25 - (P) : (((P) == 0) ? 2 : (((P) == 1) ? 3 : (((P) == 2) ? 1 : (((P) == 3) ? 0 : (((P) == 4) ? 4 : (((P) == 6) ? 7 : (((P) == 13) ? 7 : (((P) == 14) ? 3 : (((P) == 15) ? 1 : (((P) == 16) ? 2 : (((P) == 17) ? 0 : (((P) == 22) ? 1 : (((P) == 23) ? 0 : (((P) == 24) ? 4 : (((P) == 25) ? 7 : (((P) == 26) ? 4 : (((P) == 27) ? 5 : 6 )))))))))))))))))))


/*  --- Arduino Uno --- */
#elif (defined(ARDUINO_AVR_UNO) || \
       defined(ARDUINO_AVR_DUEMILANOVE) || \
       defined(__AVR_ATmega328__) || \
       defined(__AVR_ATmega328P__) || \
       defined(__AVR_ATmega328PB__))

#define UART_RX_PIN     (0) /* PD0 */
#define UART_TX_PIN     (1) /* PD1 */

#define I2C_SDA_PIN     (18) /* A4 */
#define I2C_SCL_PIN     (19) /* A5 */

#define SPI_HW_SS_PIN   (10) /* PB0 */
#define SPI_HW_MOSI_PIN (11) /* PB2 */
#define SPI_HW_MISO_PIN (12) /* PB3 */
#define SPI_HW_SCK_PIN  (13) /* PB1 */

#if defined(__AVR_ATmega328PB__)
#define __digitalPinToPortReg(P) \
(((P) >= 0 && (P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : (((P) >= 14 && (P) <= 19) ? &PORTC : &PORTE)))
#define __digitalPinToDDRReg(P) \
(((P) >= 0 && (P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : (((P) >= 14 && (P) <= 19) ? &DDRC : &DDRE)))
#define __digitalPinToPINReg(P) \
(((P) >= 0 && (P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : (((P) >= 14 && (P) <= 19) ? &PINC : &PINE)))
#define __digitalPinToBit(P) \
(((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P) - 8 : (((P) >= 14 && (P) <= 19) ? (P) - 14 : (((P) >= 20 && (P) <= 21) ? (P) - 18 : (P) - 22))))
#else
#define __digitalPinToPortReg(P) \
(((P) >= 0 && (P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define __digitalPinToDDRReg(P) \
(((P) >= 0 && (P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define __digitalPinToPINReg(P) \
(((P) >= 0 && (P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define __digitalPinToBit(P) \
(((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P) - 8 : (P) - 14))
#endif


/*  --- Other --- */
#else

#define I2C_SDA_PIN     SDA
#define I2C_SCL_PIN     SCL

#define SPI_HW_SS_PIN   SS
#define SPI_HW_MOSI_PIN MOSI
#define SPI_HW_MISO_PIN MISO
#define SPI_HW_SCK_PIN  SCK


#endif
/* #endif  //#ifndef digitalPinToPortReg */


#ifndef digitalWriteFast
#if (defined(__AVR__) || defined(ARDUINO_ARCH_AVR))
#define digitalWriteFast(P, V) \
if (__builtin_constant_p(P) && __builtin_constant_p(V)) { \
  BIT_WRITE(*__digitalPinToPortReg(P), __digitalPinToBit(P), (V)); \
} else { \
  digitalWrite((P), (V)); \
}
#else
#define digitalWriteFast digitalWrite
#endif
#endif


#ifndef pinModeFast
#if (defined(__AVR__) || defined(ARDUINO_ARCH_AVR))
#define pinModeFast(P, V) \
if (__builtin_constant_p(P) && __builtin_constant_p(V)) { \
  BIT_WRITE(*__digitalPinToDDRReg(P), __digitalPinToBit(P), (V)); \
} else { \
  pinMode((P), (V)); \
}
#else
#define pinModeFast pinMode
#endif
#endif


#ifndef digitalReadFast
#if (defined(__AVR__) || defined(ARDUINO_ARCH_AVR))
#define digitalReadFast(P) ( (int) __digitalReadFast((P)) )
#define __digitalReadFast(P ) \
  (__builtin_constant_p(P) ) ? ( \
  ( BIT_READ(*__digitalPinToPINReg(P), __digitalPinToBit(P))) ) : \
  digitalRead((P))
#else
#define digitalReadFast digitalRead
#endif
#endif

#endif /* __digitalWriteFast_h_ */

#ifndef ARDUINO_TIMER1_H
#define ARDUINO_TIMER1_H

/* set period and callback of timer1 */
void setup_timer1(long us, void(*isr)(void));

#endif /* ARDUINO_TIMER1_H */

static void noop(void) {}

/* timer interrupt handler */
static void (*__timer1__isr)(void) = &noop;

void setup_timer1(long us, void(*isr)(void)) {

  /*  initialize Timer1 */
  cli();          /*  disable global interrupts */
  TCCR1A = 0;     /*  set entire TCCR1A register to 0 */
  TCCR1B = 0;     /*  same for TCCR1B */

  /*  turn on CTC mode */
  TCCR1B |= (1 << WGM12);

  /*  1024 prescaler */
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);

  /*  each tick is 0.064 ms on 16MHz processor */
  OCR1A =  (uint16_t)((us*(F_CPU/1000000)+512)/1024-1);
  TIMSK1 |= (1 << OCIE1A);  /*  enable timer compare interrupt */

  __timer1__isr = isr;

  sei();   /*  enable global interrupts */
}

ISR(TIMER1_COMPA_vect) /*  timer compare interrupt service routine */
{
  (*__timer1__isr)();
}

#define DEFAULT_DATA_PIN  2
#define DEFAULT_CLOCK_PIN 3
#define DEFAULT_LATCH_PIN 4
#define DEFAULT_PERIOD    2000
#define NUM_DISPLAYS      8
#define NUM_BITS          8
#define FALSE 0
#define TRUE  1


static uint8_t __initialized = FALSE;   /* display initialized */
static long __period = DEFAULT_PERIOD;  /* display period for persistence */

/* pin IDs */
static uint8_t __clock_pin = DEFAULT_CLOCK_PIN;
static uint8_t __data_pin = DEFAULT_DATA_PIN;
static uint8_t __latch_pin = DEFAULT_LATCH_PIN;

/* data content and current display position */
static uint8_t __data[NUM_DISPLAYS];
static uint8_t __data_pos;

/* faster shift out function */
static void shiftOutFast(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
{
  uint8_t i;

  for (i = 0; i < NUM_BITS; i++)  {
    if (bitOrder == LSBFIRST) {
      digitalWriteFast(dataPin, !!(val & (1 << i)));
    } else {
      digitalWriteFast(dataPin, !!(val & (1 << (NUM_BITS - 1 - i))));
    }

    digitalWriteFast(clockPin, HIGH);
    digitalWriteFast(clockPin, LOW);
  }
}

/* display next LED */
static void __display_next(void) {

  /*  latch down */
  digitalWriteFast(__latch_pin, LOW);

  /*  shift out data */
  shiftOutFast(__data_pin, __clock_pin, LSBFIRST, ~__data[__data_pos]);

  /*  shift out display activation */
  uint8_t act = 1 << __data_pos;
  shiftOutFast(__data_pin, __clock_pin, LSBFIRST, act);

  /*  latch up */
  digitalWriteFast(__latch_pin, HIGH);

  /*  next slot */
  __data_pos = (__data_pos + 1) % NUM_DISPLAYS;
}

/**
 * initialize if not already initialized
 */
static void __maybe_init(void) {
  if (!__initialized) {
    setupDisplay(DEFAULT_CLOCK_PIN, DEFAULT_DATA_PIN, DEFAULT_LATCH_PIN);
  }
}

void setupDisplay(int dataPin, int latchPin, int clockPin) {
  __period = DEFAULT_PERIOD;
  __clock_pin = (uint8_t)clockPin;
  __data_pin = (uint8_t)dataPin;
  __latch_pin = (uint8_t)latchPin;

  setup_timer1(__period, &__display_next);

  pinMode(__clock_pin, OUTPUT);
  pinMode(__data_pin, OUTPUT);
  pinMode(__latch_pin, OUTPUT);

  __initialized = TRUE;
}

void displayWrite(int data, int pos) {
  __maybe_init();
  if (pos >= 0 && pos < NUM_DISPLAYS) {
    __data[pos] = (uint8_t)data;
  }
}