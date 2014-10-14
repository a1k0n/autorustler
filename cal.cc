#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "imu/imu.h"

using Eigen::Matrix3f;
using Eigen::Vector3f;

volatile bool done = false;

void handle_sigint(int signo) { done = true; }

Matrix3f SO3(Vector3f w) {
  Matrix3f m;
  m <<
      0, -w[2], w[1],
      w[2], 0, -w[0],
      -w[1], w[0], 0;
  return m;
}

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
  imu_read(i2cfd, &s);

  const Vector3f gyromean(50, -31, -17);
  Vector3f lastx(s.mag_x, s.mag_y, s.mag_z);
  // http://www.roboticsproceedings.org/rss09/p50.pdf
  Matrix3f denom;
  Vector3f num;
  while (!done) {
    timeval tv0;
    gettimeofday(&tv0, NULL);
    imu_read(i2cfd, &s);

    // angular velocity in car coords
    // car: x right, y up, z forward
    // gyro: x -roll (-z), y pitch(x), z yaw (y)
    Vector3f w(s.gyro_y, s.gyro_z, -s.gyro_x);
    // mag: x back (-z), y up(y), z right
    Vector3f x(s.mag_z, s.mag_y, -s.mag_x);
    // accel: x front (z), y left (-x), z down (-y)
    Vector3f g(-s.accel_y, -s.accel_z, s.accel_x);

    // w -= gyromean;
    Vector3f y = (x - lastx) + w.cross(x);
    Matrix3f Wi = SO3(w);
    num += Wi * y;
    denom += Wi * Wi;

    Vector3f bhat = denom.ldlt().solve(num);

    lastx = x;

    x -= bhat;
    // project magnetometer vector onto ground, as determined by the
    // accelerometer
    Vector3f proj = x - g * (x.dot(g) / (x.norm() * g.norm()));
    float angle = atan2(proj[2], proj[0]);
    printf("b:[%f %f %f] w:[%5.0f %5.0f %5.0f] x:[%4.1f %4.1f %4.1f] %0.0f\n",
           bhat[0], bhat[1], bhat[2],
           w[0], w[1], w[2],
           x[0], x[1], x[2],
           180.0 * angle / M_PI);

    timeval tv;
    gettimeofday(&tv, NULL);
    int delay = period - (tv.tv_usec % period);
    if (delay > 0)
      usleep(delay);
  }

  return 0;
}
