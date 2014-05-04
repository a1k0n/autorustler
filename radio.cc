#include <algorithm>
#include <stdlib.h>

#include "lcd/spi.h"
#include "./radio.h"

bool RadioControl::Init() {
  if (!spi.open("/dev/spidev0.0"))
    return false;
  return true;
}

bool RadioControl::GetRadioState(RCState* state) {
  uint8_t txbuf[2], rxbuf[32];
  txbuf[0] = 0x02;  // get radio state cmd
  txbuf[1] = 0;     // ignored
  spi.xfer(txbuf, rxbuf, 2);
  // rxbuf[1] is number of queued samples
  uint8_t readlen = std::min(
      (uint8_t)(sizeof(rxbuf)/2),
      rxbuf[1]);
  if (readlen == 0) {
    return false;
  }
  spi.xfer(NULL, rxbuf, readlen*2);
  // ignore all but the most recent input
  state->throttle = rxbuf[(readlen-1)*2];
  state->steering = rxbuf[(readlen-1)*2 + 1];
  return true;
}

bool RadioControl::GetBatteryVoltage(float *voltage) {
  const float kR1 = 78.85;  // FIXME: re-measure these
  const float kR2 = 10.01;
  const float kVRef = 1.1;
  uint8_t txbuf[3] = {0x05, 0, 0}, rxbuf[3];
  if (spi.xfer(txbuf, rxbuf, 3) != 3)
    return false;
  uint16_t adc = (rxbuf[2]<<8) + rxbuf[1];
  *voltage = (kR1 + kR2) * kVRef * adc / (1024.0 * kR2);
  return true;
}
