#ifndef UI_UISTATE_H_
#define UI_UISTATE_H_

#include "imu/imu.h"
#include "car/radio.h"
#include "ui/recording.h"

struct UIState {
  bool done;
  bool is_recording;

  uint8_t cam_preview[64*48];
  int32_t gps_x, gps_y, gps_z;
  IMUState imu_state;
  RCState rc_state;
  float vbat;
  int gps_SVs;
};

extern UIState uistate;
extern Recording recording;

#endif  // UI_UISTATE_H_
