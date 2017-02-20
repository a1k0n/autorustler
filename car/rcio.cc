#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include "car/radio.h"

bool RadioControl::Open(char *devfname) {
  fd_ = open(devfname, O_RDWR);
  if (fd_ == -1) {
    perror(devfname);
    return false;
  }

  uint8_t kick = 0;  // wake up device with a null byte
  write(fd_, &kick, 1);

  return true;
}

void RadioControl::SetOutputState(const RCState &state) {
  uint8_t ch1 = state.steering;
  uint8_t ch2 = state.throttle;
  uint8_t buf[5] = { 0x5a, 0xa5, ch1, ch2, ~(0x5a+0xa5+ch1+ch2) };
  write(fd_, buf, sizeof(buf));
}

RadioControl::RadioControl() {
  sequence_ = 0;
  ch1_ = 0, ch2_ = 0;
  cksum_ = 0;
  proto_state_ = 0;
}

bool RadioControl::GetRadioState(RCState *state) {
  // read serial data input from stdin
  uint8_t buf[16];

  if (fd_ == -1) {
    return false;
  }

  int n = read(fd_, buf, sizeof(buf));
  if (n <= 0) {
    fd_ = -1;
    return false;
  }

  bool newstate = false;

  // a5 5a eb 83 79 19 00 00
  for (int i = 0; i < n; i++) {
    uint8_t c = buf[i];
    cksum_ += c;
    switch (proto_state_) {
      case 0:
        if (c == 0xa5) proto_state_++;
        break;
      case 1:
        if (c != 0x5a) {
          proto_state_ = 0;
          cksum_ = 0;
        } else if (c == 0xa5) {
          cksum_ = 0xa5;
          break;
        } else {
          proto_state_ = 2;
        }
        break;
      case 2:
        sequence_ = c;
        proto_state_++;
        break;
      case 3:
        ch1_ = c;
        proto_state_++;
        break;
      case 4:
        ch2_ = c;
        proto_state_++;
        break;
      case 5:
        if (cksum_ == 255) {
          state->steering = ch1_;
          state->throttle = ch2_;
          newstate = true;
        // } else {
          // fprintf(stderr, "# bad cksum %d %d\n", cksum_ - c, ~c);
        }
        proto_state_ = 0;
        cksum_ = 0;
        break;
    }
  }
  return newstate;
}

// not supported (yet?)
bool RadioControl::GetBatteryVoltage(float *voltage) {
  return false;
}

