#include <stdlib.h>
#include <algorithm>

#include "gpio/spi.h"
#include "car/radio.h"

bool RadioControl::Init() {
  if (!spi.open("/dev/spidev0.0"))
    return false;
  return true;
}

bool RadioControl::GetRadioState(RCState* state) {
  // padding + get radio state cmd + 2 bytes output
  uint8_t txbuf[5] = {0, 0, 0x02, 0, 0}, rxbuf[5];
  if (spi.xfer(txbuf, rxbuf, 5) != 5)
    return false;
  // should never be outside this range; must be a comm problem?
  if (rxbuf[3] < 60 || rxbuf[3] > 170 ||
      rxbuf[4] < 60 || rxbuf[4] > 170)
    return false;
  state->throttle = rxbuf[3];
  state->steering = rxbuf[4];
  return true;
}

bool RadioControl::GetBatteryVoltage(float *voltage) {
  const float kR1 = 78.85;  // FIXME: re-measure these
  const float kR2 = 10.01;
  const float kVRef = 1.1;
  // padding + get battery voltage + 2 bytes output
  uint8_t txbuf[5] = {0, 0, 0x05, 0, 0}, rxbuf[5];
  if (spi.xfer(txbuf, rxbuf, 5) != 5)
    return false;
  uint16_t adc = (rxbuf[4] << 8) + rxbuf[3];
  if (adc > 1023)  // FIXME: sometimes we get junk
    return false;
  *voltage = (kR1 + kR2) * kVRef * adc / (1024.0 * kR2);
  return true;
}
