#ifndef CAR_RADIO_H_
#define CAR_RADIO_H_

// #include "gpio/spi.h"
#include <stdint.h>

struct RCState {
  uint8_t throttle, steering;

  RCState() { throttle = steering = 0; }
};

class RadioControl {
 public:
  RadioControl();

  bool Open(char *devfname);

  // returns true if new state is available; otherwise, state is unmodified
  // since last poll
  bool GetRadioState(RCState* state);

  // enables radio control, which passes radio state through to throttle and
  // steering inputs (this is the power-on default)
  bool EnableRadioControl();
  // disables radio control, and sets throttle and steering outputs
  void SetOutputState(const RCState& state);

  bool GetBatteryVoltage(float *voltage);

  int GetFileDescriptor() { return fd_; }  // for select() polling

 private:
  // SPIDev spi;
  int fd_;
  uint8_t sequence_, ch1_, ch2_, cksum_, proto_state_;
};

#endif  // CAR_RADIO_H_
