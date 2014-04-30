#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include "./gpio.h"
#include "./lcd.h"
#include "./spi.h"

int main() {
  if (!gpio_init())
    return -1;
  LCD lcd;
  if (!lcd.init()) {
    fprintf(stderr, "lcd init fail!");
    return -1;
  }

  uint8_t screen[84*6];
  int t = 0;
  for (;; t++) {
    int idx = 0;
    float r = 10*sin(t*0.1) + 22;
    r *= r;
    for (int y = 0; y < 48; y += 8) {
      for (int x = 0; x < 84; x++) {
        uint8_t b = 0;
        for (int yy = 0; yy < 8; yy++) {
          b >>= 1;
          if ((y+yy-24)*(y+yy-24)*1.2*1.2 + (x-42)*(x-42) < r) {
            if ((x+yy+t)&1)
              b |= 0x80;
          }
          if ((y+yy-24)*(y+yy-24)*1.2*1.2 + (x-42)*(x-42) < r/2) {
            b |= 0x80;
          }
        }
        screen[idx++] = b;
      }
    }
    lcd.gotoxy(0,0);
    lcd.draw(screen, 84*6);
    usleep(30000);
  }
}
