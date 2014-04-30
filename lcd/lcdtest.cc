#include <stdint.h>
#include <stdio.h>
#include "./gpio.h"
#include "./lcd.h"
#include "./spi.h"

int main() {
  if (!gpio_init())
    return -1;
  LCD lcd;
  if (!lcd.init()) {
    return -1;
  }

  uint8_t screen[84*6];
  int idx = 0;
  for (int y = 0; y < 48; y += 8) {
    for (int x = 0; x < 84; x++) {
      uint8_t b = 0;
      for (int yy = 0; yy < 8; yy++) {
        b >>= 1;
        if ((y+yy-24)*(y+yy-24) + (x-42)*(x-42) < 22*22)
          b |= 0x80;
      }
      screen[idx++] = b;
    }
  }
  lcd.draw(screen, 84*6);
}
