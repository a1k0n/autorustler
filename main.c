#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


int main() {
  DDRC = 1<<5;
  for (;;) {
    PORTC ^= 1<<5;
    _delay_ms(200);
  }
}
