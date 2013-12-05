#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

static uint8_t spi_state = 0;
static uint8_t spi_hibyte;
ISR(SPI_STC_vect) {
  char spi_in = SPDR;
  switch (spi_state) {
    default:  // command mode, state 0 (or invalid states)
      if (spi_in == 0x01) {  // command 0x01: read timer
        SPDR = TCNT1L;
        spi_hibyte = TCNT1H;
        spi_state = 1;
      } else {
        SPDR = 0;
      }
      break;
    case 1: // send one high byte, then return to state 0
      SPDR = spi_hibyte;
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

  sei();

  DDRC = 1<<5;
  for (;;) {
    PORTC ^= 1<<5;
    _delay_ms(200);
  }
}
