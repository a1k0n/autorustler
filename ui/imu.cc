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


  IMUState s;
  RCState rcstate;
  float vbat;

  while (!uistate.done) {
    timeval tv0;
    gettimeofday(&tv0, NULL);
    imu_read(i2cfd, &s);
    if (!rc.GetRadioState(&rcstate)) {
      rc.GetRadioState(&rcstate);
    }
    if (!rc.GetBatteryVoltage(&vbat))
      rc.GetBatteryVoltage(&vbat);

    memcpy(const_cast<IMUState*>(&uistate.imu_state), &s, sizeof(s));
    memcpy(const_cast<RCState*>(&uistate.rc_state), &rcstate, sizeof(rcstate));
    uistate.vbat = vbat;

    if (uistate.is_recording) {
      RecordHeader rh;
      rh.Init(sizeof(s) + sizeof(rcstate) + sizeof(vbat), 2);
      recording.StartWriting();
      recording.Write(reinterpret_cast<uint8_t*>(&rh), sizeof(rh));
      recording.Write(reinterpret_cast<uint8_t*>(&s), sizeof(s));
      recording.Write(reinterpret_cast<uint8_t*>(&rcstate), sizeof(rcstate));
      recording.Write(reinterpret_cast<uint8_t*>(&vbat), sizeof(vbat));
      recording.StopWriting();
    }
#if 0
    printf("%d.%06d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %0.3f\n",
           tv0.tv_sec, tv0.tv_usec,
           s.gyro_x, s.gyro_y, s.gyro_z,
           s.mag_x, s.mag_y, s.mag_z,
           s.accel_x, s.accel_y, s.accel_z,
           rcstate.steering, rcstate.throttle, vbat);
#endif

    timeval tv;
    gettimeofday(&tv, NULL);
    int delay = 20000 - (tv.tv_usec % 20000);
    if (delay > 0)
      usleep(delay);
  }
}
