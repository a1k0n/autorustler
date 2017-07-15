#ifndef CAR_TEENSY_H_
#define CAR_TEENSY_H_

#include "gpio/i2c.h"

// i2c-connected teensy running a program to write to servo / ESC, read from
// encoders & analog servo position feedback

class Teensy {
 public:
  explicit Teensy(const I2C &i2cbus);

  bool Init();

  bool SetControls(uint8_t led, int8_t esc, int8_t servo);
  bool GetFeedback(uint8_t *servo, uint16_t *encoders);

 private:
  const I2C &i2c_;
};

#endif  // CAR_TEENSY_H_

