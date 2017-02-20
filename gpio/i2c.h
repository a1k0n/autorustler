#ifndef GPIO_I2C_H_
#define GPIO_I2C_H_

#include <stdint.h>

class I2C {
 public:
  ~I2C() { Close(); }

  bool Open();
  void Close();

  bool Write(uint8_t addr, uint8_t reg, uint8_t value) const;
  bool Read(uint8_t addr, uint8_t reg, int len, uint8_t *outbuf) const;

 private:
  int fd_;
};

#endif  // GPIO_I2C_H_
