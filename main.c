#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>


PROGMEM const char msg[] = "Hello world";
char ringptr = 0;
ISR(SPI_STC_vect) {
  char x = SPDR;
  SPDR = pgm_read_byte(msg+ringptr);
  ringptr++;
  if (ringptr == 11)
    ringptr = 0;
}

void spi_init() {
  // PB3 (MOSI), PB5 (SCK), PB2 (/SS) input
  // PB4 (MISO) output
  DDRB |= 0x10;
  // enable SPI
  SPCR = (1<<SPE) | (1<<SPIE);
}

int main() {
  spi_init();

  sei();

  DDRC = 1<<5;
  for (;;) {
    PORTC ^= 1<<5;
    _delay_ms(200);
  }
}
