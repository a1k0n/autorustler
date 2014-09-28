#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "car/radio.h"
#include "imu/imu.h"

volatile bool done = false;

void handle_sigint(int signo) { done = true; }

int main() {
  int i2cfd = open("/dev/i2c-1", O_RDWR);
  if (i2cfd == -1) {
    perror("/dev/i2c-1");
    return 1;
  }

  signal(SIGINT, handle_sigint);

  imu_init(i2cfd);
  printf("# t gx gy gz mx my mz ax ay az steer throttle vbat\n");

  RadioControl rc;
  if (!rc.Init()) {
    fprintf(stderr, "radio init fail\n");
    return 1;
  }

  float vbat;
  imu_state s;
  RCState rcstate;
  while (!done) {
    timeval tv0;
    gettimeofday(&tv0, NULL);
    imu_read(i2cfd, &s);
    if (!rc.GetRadioState(&rcstate)) {
      printf("retrying radio\n");
      rc.GetRadioState(&rcstate);
    }
    if (!rc.GetBatteryVoltage(&vbat))
      rc.GetBatteryVoltage(&vbat);
#if 0
    fprintf(stderr, "gyro [%+4d %+4d %+4d] mag [%+4d %+4d %+4d] "
            "acc [%+4d %+4d %+4d]\e[K\r",
            s.gyro_x, s.gyro_y, s.gyro_z,
            s.mag_x, s.mag_y, s.mag_z,
            s.accel_x, s.accel_y, s.accel_z);
    fflush(stderr);
#endif
    printf("%d.%06d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %0.3f\n",
           tv0.tv_sec, tv0.tv_usec,
           s.gyro_x, s.gyro_y, s.gyro_z,
           s.mag_x, s.mag_y, s.mag_z,
           s.accel_x, s.accel_y, s.accel_z,
           rcstate.steering, rcstate.throttle, vbat);
    fflush(stdout);
    timeval tv;
    gettimeofday(&tv, NULL);
    int delay = 20000 - (tv.tv_usec % 20000);
    if (delay > 0)
      usleep(delay);
  }

  return 0;
}
