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
  state->throttle = rxbuf[(readlen-1)*2 + 1];
  state->steering = rxbuf[(readlen-1)*2];
  return true;
}
