#include <stdint.h>
#include <unistd.h>
#include "./gpio.h"
#include "./lcd.h"
#include "./spi.h"

const int LCD_RST_GPIO = 27;
const int LCD_DC_GPIO = 17;

bool LCD::Init() {
  if (!spi.open("/dev/spidev0.1"))
    return false;

  uint8_t txbuf[8];
  INP_GPIO(LCD_RST_GPIO);
  OUT_GPIO(LCD_RST_GPIO);
  INP_GPIO(LCD_DC_GPIO);
  OUT_GPIO(LCD_DC_GPIO);
  GPIO_CLR = 1 << LCD_RST_GPIO;
  usleep(10);
  GPIO_SET = 1 << LCD_RST_GPIO;
  usleep(10);

  GPIO_CLR = 1 << LCD_DC_GPIO;  // command mode
  txbuf[0] = 0x21;  // turn on chip, horizontal addressing, extended insn set
  txbuf[1] = 0xBC;  // set LCD Vop
  txbuf[2] = 0x04;  // set temp coeff
  txbuf[3] = 0x14;  // LCD bias mode
  txbuf[4] = 0x20;  // turn on chip, horizontal addressing, basic insn set
  txbuf[5] = 0x0c;  // normal mode (not blank or all on)
  txbuf[6] = 0x40;  // y = 0
  txbuf[7] = 0x80;  // x = 0
  if (!spi.xfer(txbuf, NULL, 8))
    return false;

  return true;
}

void LCD::GotoXY(int x, int y) {
  uint8_t txbuf[2];
  txbuf[0] = 0x40 + y;
  txbuf[1] = 0x80 + x;
  GPIO_CLR = 1 << LCD_DC_GPIO;  // command mode
  spi.xfer(txbuf, NULL, 2);
}

void LCD::Draw(const uint8_t* framebuf, int len) {
  GPIO_SET = 1 << LCD_DC_GPIO;  // data mode
  spi.xfer(framebuf, NULL, len);
}

int LCD::WriteString(int x, int y, const char *str, uint8_t *framebuf) {
  // TODO(a1k0n): bit shifts to handle y&7 != 0
  y /= 8;
  int bufptr = y*84 + x;
  int x0 = x;
  while (*str && x < 84) {
    if (x != x0) {
      // add a single-pixel space between each character
      x++;
      framebuf[bufptr++] = 0;
    }
    uint8_t c = *str - 0x20;
    if (c > 95) continue;
    const uint8_t *glyph = LCD::font[c];
    for (int i = 0; i < 5; i++, x++) {
      if (x < 0) continue;
      if (x >= 84) break;
      framebuf[bufptr++] = glyph[i];
    }
    str++;
  }
  return x - x0;
}
