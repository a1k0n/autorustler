#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "car/radio.h"
#include "imu/imu.h"
#include "ui/uistate.h"

void* IMUThread(void* data) {
  int i2cfd = reinterpret_cast<int>(data);

  RadioControl rc;

  if (imu_init(i2cfd)) {
    fprintf(stderr, "imu init fail\n");
    exit(1);
    return NULL;
  }

  if (!rc.Init()) {
    fprintf(stderr, "radio init fail\n");
    exit(1);
    return NULL;
  }


  IMUState imu;
  RCState rcstate;
  float vbat;

  while (!uistate.done) {
    timeval tv0;
    gettimeofday(&tv0, NULL);
    imu_read(i2cfd, &imu);
    if (!rc.GetRadioState(&rcstate)) {
      rc.GetRadioState(&rcstate);
    }
    if (!rc.GetBatteryVoltage(&vbat))
      rc.GetBatteryVoltage(&vbat);

    memcpy(const_cast<IMUState*>(&uistate.imu_state), &imu, sizeof(imu));
    memcpy(const_cast<RCState*>(&uistate.rc_state), &rcstate, sizeof(rcstate));
    uistate.vbat = vbat;

    timeval tv;
    gettimeofday(&tv, NULL);
    int delay = 20000 - (tv.tv_usec % 20000);
    if (delay > 0)
      usleep(delay);
  }
}
