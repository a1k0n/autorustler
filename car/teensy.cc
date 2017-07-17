#include "car/teensy.h"

static const int TEENSY_ADDRESS = 118;

Teensy::Teensy(const I2C &i2cbus) : i2c_(i2cbus) {}

bool Teensy::Init() {
  return i2c_.Write(TEENSY_ADDRESS, 0x00, 0);
}

bool Teensy::SetControls(uint8_t led, int8_t esc, int8_t servo) {
  uint8_t buf[3] = {led, esc, servo};
  return i2c_.Write(TEENSY_ADDRESS, 0, 3, buf);
}

bool Teensy::GetFeedback(uint8_t *servo, uint16_t *encoders) {
  uint8_t buf[17];
  if (!i2c_.Read(TEENSY_ADDRESS, 7, 9, buf)) {
    return false;
  }

  *servo = buf[0];
  for (int i = 0; i < 4; i++) {
    encoders[i] = buf[1 + 2*i] + (buf[2 + 2*i] << 8);
  }

  return true;
}
