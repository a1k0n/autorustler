#ifndef RADIO_H_
#define RADIO_H_

#include "lcd/spi.h"

struct RCState {
  uint8_t throttle, steering;

  RCState() { throttle = steering = 0; }
};

class RadioControl {
 public:
  bool Init();

  // returns true if new state is available; otherwise, state is unmodified
  // since last poll
  bool GetRadioState(RCState* state);

  // enables radio control, which passes radio state through to throttle and
  // steering inputs (this is the power-on default)
  bool EnableRadioControl();
  // disables radio control, and sets throttle and steering outputs
  void SetOutputState(const RCState& state);

  bool GetBatteryVoltage(float *voltage);

 private:
  SPIDev spi;
};

#endif  // RADIO_H_
