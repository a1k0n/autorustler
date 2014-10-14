#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include <Eigen/Dense>
#include "imu/imu.h"

using Eigen::Vector3f;

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
  Vector3f b(0, 0, 0);
  imu_read(i2cfd, &s);

  const Vector3f gyromean(49.68937876, -31.1743487, -16.9739479);
  Vector3f lastm(s.mag_x, s.mag_y, s.mag_z);
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
    Vector3f m(s.mag_x - b[0], s.mag_y - b[1], s.mag_z - b[2]);
    Vector3f dm(m - lastm);
    float dmmag = dm.norm();
    if (dmmag > 10) {
      float scale = gain * (m.norm() - lastm.norm()) / dmmag;
      b += scale * dm;
      printf("b:[%f %f %f] m:[%f %f %f]\n", b[0], b[1], b[2],
             m[0], m[1], m[2]);
    }

    lastm = m;

    timeval tv;
    gettimeofday(&tv, NULL);
    int delay = period - (tv.tv_usec % period);
    if (delay > 0)
      usleep(delay);
  }

  return 0;
}
