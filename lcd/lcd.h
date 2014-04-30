#ifndef LCD_LCD_H_
#define LCD_LCD_H_

#include "./spi.h"

class LCD {
 public:
  bool init();

  void gotoxy(int x, int y);
  void draw(const uint8_t* framebuf, int len);

 private:
  SPIDev spi;
};

#endif  // LCD_LCD_H_
