#ifndef LCD_LCD_H_
#define LCD_LCD_H_

#include "./spi.h"

class LCD {
 public:
  bool Init();

  void GotoXY(int x, int y);
  void Draw(const uint8_t* framebuf, int len);

  // Drawing routines; modifies framebuffer but not LCD screen.

  // Draw a string into a framebuffer starting at upper-left coordinate x,y
  // returns number of horizontal pixels spanned (doesn't do word wrapping)
  static int WriteString(int x, int y, const char *str, uint8_t* framebuf);

 private:
  SPIDev spi;

  static const uint8_t font[][5];
};

#endif  // LCD_LCD_H_
