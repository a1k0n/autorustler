#ifndef LCD_GPIO_H_
#define LCD_GPIO_H_

// I/O access
extern volatile unsigned *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or
// SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7 << (((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1 << (((g)%10)*3))
#define SET_GPIO_ALT(g, a) *(gpio+(((g)/10))) |= \
            (((a) <= 3?(a)+4:(a) == 4?3:2) << (((g)%10)*3))

#define GPIO_SET *(gpio+7)   // sets bits which are 1
#define GPIO_CLR *(gpio+10)  // clears bits which are 1

bool gpio_init();  // N.B. must be root

#endif  // LCD_GPIO_H_
