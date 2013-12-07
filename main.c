#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

volatile static uint16_t ch1_counts = 0, ch2_counts = 0;
volatile static uint8_t pwm_mirror = 1;
ISR(PCINT0_vect) {
  static uint16_t ch1_start = 0, ch2_start = 0;
  static uint8_t ch1_state = 0, ch2_state = 0;
  uint16_t tm = TCNT1L;
  uint8_t state = PINB;
  tm |= (uint16_t)TCNT1H << 8;
  if ((state&1) != ch1_state) {
    ch1_state = state&1;
    if (ch1_state) {
      ch1_start = tm;
    } else {
      ch1_counts = (tm - ch1_start);
      if (pwm_mirror) {
        OCR0A = ch1_counts>>8;
      }
    }
  }
  if ((state&2) != ch2_state) {
    ch2_state = state&2;
    if (ch2_state) {
      ch2_start = tm;
    } else {
      ch2_counts = (tm - ch1_start);
      if (pwm_mirror) {
        OCR0B = ch2_counts>>8;
      }
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
  char spi_in = SPDR;
  static uint8_t spi_state = 0;
  static uint8_t spi_hibyte;
  switch (spi_state) {
    default:  // command mode, state 0 (or invalid states)
      if (spi_in == 0x01) {  // command 0x01: read timer
        SPDR = TCNT1L;
        spi_hibyte = TCNT1H;
        spi_state = 1;
      } else if(spi_in == 0x02) {  // command 0x02: read channels
        SPDR = ch1_counts&255;
        spi_state = 2;
      } else {
        SPDR = 0;
      }
      break;
    case 1: // send one high byte, then return to state 0
      SPDR = spi_hibyte;
      spi_state = 0;
      break;
    case 2: // read counts, ch1 hi
      SPDR = ch1_counts >> 8;
      spi_state++;
      break;
    case 3: // read counts, ch2 lo
      SPDR = ch2_counts & 255;
      spi_state++;
      break;
    case 4: // read counts, ch2 hi
      SPDR = ch2_counts >> 8;
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
  // Set up TMR1 to count up on each clock cycle (20MHz)
  TCCR1B = 0x01;

  spi_init();
  pwmcapture_init();
  pwmgen_init();

  sei();

  DDRC = 1<<5;
  for (;;) {
    PORTC ^= 1<<5;
    _delay_ms(200);
  }
}
