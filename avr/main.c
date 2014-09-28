#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>

static uint16_t battery_voltage = 0;

volatile static uint8_t ch1_counts = 0, ch2_counts = 0;
volatile static uint8_t pwm_mirror = 1;
ISR(PCINT0_vect) {
  static uint8_t ch1_start = 0, ch2_start = 0;
  static uint8_t ch1_state = 0, ch2_state = 0;
  uint8_t tm = TCNT0;
  uint8_t state = PINB;
  if ((state&1) != ch1_state) {
    ch1_state = state&1;
    if (ch1_state) {
      ch1_start = tm;
    } else {
      ch1_counts = (tm - ch1_start);
    }
  }
  if ((state&2) != ch2_state) {
    ch2_state = state&2;
    if (ch2_state) {
      ch2_start = tm;
    } else {
      ch2_counts = (tm - ch1_start);
    }
  }
}

// dumb int-on-pin-change pwm capture
static void pwmcapture_init() {
  PORTB = 0x03;  // add pull-ups to inputs
  PCMSK0 = 0x03;
  PCICR = 0x01;
}

ISR(SPI_STC_vect) {
  uint8_t spi_in = SPDR;
  static uint8_t spi_state = 0, spi_hibyte = 0;
  static uint8_t spi_readlen = 0;
  switch (spi_state) {
    default:  // command mode, state 0 (or invalid states)
      if (spi_in == 0x01) {  // command 0x01: read timer
        SPDR = TCNT1L;
        spi_hibyte = TCNT1H;
        spi_state = 1;
      } else if (spi_in == 0x02) {  // command 0x02: read PWM input state
        SPDR = ch1_counts;
        spi_hibyte = ch2_counts;
        spi_state = 1;
      } else if (spi_in == 0x03) {  // command 0x03: set pwm mirroring
        SPDR = 1 + (pwm_mirror << 1);
        spi_state = 5;
      } else if (spi_in == 0x04) {  // command 0x04: set pwm output
        SPDR = 1;
        spi_state = 6;
      } else if (spi_in == 0x05) {  // command 0x05: get battery voltage
        SPDR = battery_voltage & 0xff;
        spi_hibyte = battery_voltage >> 8;
        spi_state = 1;
      } else {  // unknown command, or padding (command 0)
        SPDR = 0;
      }
      break;
    case 1: // send one high byte, then return to state 0
      SPDR = spi_hibyte;
      spi_state = 0;
      break;
    case 5: // set PWM mirror
      pwm_mirror = spi_in;
      SPDR = 0;
      spi_state = 0;
      break;
    case 6: // set PWM output ch1
      SPDR = OCR0A;
      OCR0A = spi_in;
      spi_state++;
      break;
    case 7: // set PWM output ch2
      SPDR = OCR0B;
      OCR0B = spi_in;
      spi_state = 0;
      break;
  }
}

static void spi_init() {
  // PB3 (MOSI), PB5 (SCK), PB2 (/SS) input
  // PB4 (MISO) output
  DDRB |= 0x10;
  // enable SPI
  SPCR = (1<<SPE) | (1<<SPIE);
}

static void pwmgen_init() {
  // Generate PWM on PD6/PD5 (OC0A/OC0B respectively)
  // ugh, this is only 8-bit PWM, though, and at ~300Hz.  i hope that
  // works.
  // set TMR0 prescaler to /256
  DDRD |= (1<<5) | (1<<6);

  // period in ms = 65536/F_CPU
  // duration of N counts (milliseconds) = T = 256000*n/F_CPU
  // n = T*F_CPU/256000
  OCR0A = (1.5*F_CPU/256000);
  OCR0B = (1.5*F_CPU/256000);
  // (at 20MHz this gives a dynamic range of 78-156, 117 being center)
  // so ~6.3 bit pwm...

  TCCR0A = 0xA3; // enable OC0A, OC0B, fast PWM mode
  TCCR0B = 0x04; // prescaler /256, 305.1Hz (3.2768ms cycle)
}

int main() {
  // enable WDT
  wdt_enable(WDTO_15MS);

  // Set up TMR1 to count up on each clock cycle (20MHz)
  TCCR1B = 0x01;

  // set up ADC on channel 0
  PORTC = 0;
  DDRC = 1<<3;
  DIDR0 = 0x01;
  ADMUX = 0xc0;  // internal 1.1V ref, select ADC0
  ADCSRA = 0x87;  // enable ADC

  spi_init();
  pwmcapture_init();
  pwmgen_init();

  sei();

  uint32_t itercount = 0;
  for (;;) {
    itercount++;
    wdt_reset();
    if (!(itercount&0x7ffff)) {
      PORTC ^= 1<<3;
      ADCSRA |= _BV(ADSC);
      while(!bit_is_set(ADCSRA,ADIF));
      ADCSRA |= _BV(ADIF);
      battery_voltage = ADC;
    }
    if (pwm_mirror && ch1_counts != 0 && ch2_counts != 0) {
      OCR0A = ch1_counts;
      OCR0B = ch2_counts;
    }
  }
}
