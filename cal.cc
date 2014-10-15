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

// returns 3x3 rotation matrix which gives the derivative of a direction w.r.t.
// time and angular velocity vector w
Matrix3f SO3(Vector3f w) {
  Matrix3f m;
  m << 0, -w[2], w[1],
       w[2], 0, -w[0],
       -w[1], w[0], 0;
  return m;
}

// calibration constants determined empirically from temperature sensitivity
// b is the offset at temp=0
// m is the change in offset per lsb
// b [-182.11435759 -179.31842819 -212.82507905]
// m [-0.01169129 -0.01123162 -0.01164203]
const Vector3f gyrocal_b(-182, -179, -213);
const Vector3f gyrocal_m(-0.01169, -0.01123, -0.01164);

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
    Vector3f w0(s.gyro_y, s.gyro_z, -s.gyro_x);
    // scale to radians/second
    w0 -= gyrocal_b + s.gyro_temp * gyrocal_m;
    Vector3f w = w0 * M_PI * 2000.0 / (180.0 * 32767.0);
    // mag: x back (-z), y up(y), z right
    Vector3f x(s.mag_z, s.mag_y, -s.mag_x);
    // accel: x front (z), y left (-x), z down (-y)
    Vector3f g(-s.accel_y, -s.accel_z, s.accel_x);

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
    printf("%0.1f C b:[%f %f %f] w:[%5.0f %5.0f %5.0f] x:[%4.1f %4.1f %4.1f] "
           "%0.0f\n",
           (s.gyro_temp - -13200) * (1.0 / 280.0) + 35.0,
           bhat[0], bhat[1], bhat[2],
           w0[0], w0[1], w0[2],
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
