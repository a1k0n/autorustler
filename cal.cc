#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>

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

  IMUState s;
  const int period = 50000;
  const float gain = 1.0;
  float b[3] = {0, 0, 0};
  imu_read(i2cfd, &s);
  float lastm[3] = {s.mag_x, s.mag_y, s.mag_z};
  while (!done) {
    timeval tv0;
    gettimeofday(&tv0, NULL);
    imu_read(i2cfd, &s);

    // some sort of kalman filter
    // gyro has a slight offset
    // magnetometer has a huge offset
    // there may be cross-axis misalignment
    // g' ~ N(g + bg, sigmag)
    // m' ~ N(m + bm, sigmam)

    // float m[3] = {s.mag_x - b[0], s.mag_y - b[1], s.mag_z - b[2]};
    float m[3] = {s.mag_x - b[0], s.mag_y - b[1], s.mag_z - b[2]};
    float dm[3] = {m[0] - lastm[0], m[1] - lastm[1], m[2] - lastm[2]};
    float dmmag = dm[0]*dm[0] + dm[1]*dm[1] + dm[2]*dm[2];
    if (dmmag > 10) {
      float mag2 = sqrt(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
      float mag1 = sqrt(lastm[0]*lastm[0] + lastm[1]*lastm[1] +
                        lastm[2]*lastm[2]);
      float scale = gain * (mag2 - mag1) / sqrt(dmmag);
      b[0] += dm[0] * scale;
      b[1] += dm[1] * scale;
      b[2] += dm[2] * scale;
      printf("b:[%f %f %f] m:[%f %f %f]\n", b[0], b[1], b[2],
             m[0], m[1], m[2]);
    }

    lastm[0] = m[0];
    lastm[1] = m[1];
    lastm[2] = m[2];

    timeval tv;
    gettimeofday(&tv, NULL);
    int delay = period - (tv.tv_usec % period);
    if (delay > 0)
      usleep(delay);
  }

  return 0;
}
