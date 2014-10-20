#ifndef GPIO_I2C_H_
#define GPIO_I2C_H_

#include <stdint.h>

extern int i2c_write(int file, uint8_t addr, uint8_t reg, uint8_t value);
extern int i2c_read(int file, uint8_t addr, uint8_t reg,
                    uint8_t *outbuf, int len);

#endif  // GPIO_I2C_H_
