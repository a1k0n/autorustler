#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

volatile static uint16_t ch1_counts = 0, ch2_counts = 0;
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
      ch1_counts = tm - ch1_start;
    }
  }
  if ((state&2) != ch2_state) {
    ch2_state = state&2;
    if (ch2_state) {
      ch2_start = tm;
    } else {
      ch2_counts = tm - ch1_start;
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

static void tmr1_init() {
  TCCR1B = 0x01;
}

static void spi_init() {
  // PB3 (MOSI), PB5 (SCK), PB2 (/SS) input
  // PB4 (MISO) output
  DDRB |= 0x10;
  // enable SPI
  SPCR = (1<<SPE) | (1<<SPIE);
}

int main() {
  tmr1_init();
  spi_init();
  pwmcapture_init();

  sei();

  DDRC = 1<<5;
  for (;;) {
    PORTC ^= 1<<5;
    _delay_ms(200);
  }
}
