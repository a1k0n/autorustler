#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include "./imu.h"

int main() {
  int i2cfd = open("/dev/i2c-1", O_RDWR);
  if (i2cfd == -1) {
    perror("/dev/i2c-1");
    return 1;
  }

  imu_init(i2cfd);

  for (;;) {
    imu_state s;
    imu_read(i2cfd, &s);
    fprintf(stderr, "gyro [%+4d %+4d %+4d] mag [%+4d %+4d %+4d] "
            "acc [%+4d %+4d %+4d]\e[K\r",
            s.gyro_x, s.gyro_y, s.gyro_z,
            s.mag_x, s.mag_y, s.mag_z,
            s.accel_x, s.accel_y, s.accel_z);
    fflush(stderr);
    usleep(100000);
  }

  return 0;
}
