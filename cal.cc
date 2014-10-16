#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include "imu/imu.h"

using Eigen::Matrix4f;
using Eigen::Vector3f;
using Eigen::Vector4f;

typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

volatile bool done = false;

void handle_sigint(int signo) { done = true; }

// calibration constants determined empirically from temperature sensitivity
// b is the offset at temp=0
// m is the change in offset per lsb
// b [-182.11435759 -179.31842819 -212.82507905]
// m [-0.01169129 -0.01123162 -0.01164203]
const Vector3f gyrocal_b(-182.11, -179.32, -212.83);
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
  imu_read(i2cfd, &s);

  Matrix4f XTX = 0.1 * Matrix4f::Identity();
  Vector4f XTY = Vector4f::Zero();

  // our state is [nx ny nz bx by bz] where n is the unbiased magnetometer
  // reading, i.e., n = m - b, and b is the magnetometer bias bias.
  // prediction: n' = R(w) * n, b' = b, so F = [[R(w) 0] [0 I]]
  // measurement: z = n + b, so H = [I I]
  while (!done) {
    timeval tv0;
    gettimeofday(&tv0, NULL);
    imu_read(i2cfd, &s);

#if 0
    // angular velocity in car coords
    // car: x right, y up, z forward
    // gyro: x roll (z), y -pitch(-x), z -yaw (-y)
    Vector3f w0(-s.gyro_y, -s.gyro_z, s.gyro_x);
    // scale to radians/second
    w0 -= gyrocal_b + s.gyro_temp * gyrocal_m;
    // datasheet says scale factor is 14.375 LSB/deg/s
    Vector3f w = w0 * M_PI / (180.0 * 14.375);
    // accel: x front (z), y left (-x), z down (-y)
    Vector3f g(-s.accel_y, -s.accel_z, s.accel_x);
#endif

    Vector4f b(s.mag_x, s.mag_y, s.mag_z, 1);
    XTX += b * b.transpose();
    float bmag = b.squaredNorm();
    XTY += Vector4f(bmag * b[0], bmag * b[1], bmag * b[2], bmag);

    Vector4f beta = XTX.ldlt().solve(XTY);
    Vector3f bias = beta.block<3, 1>(0, 0) / 2;
    bmag = sqrt(b[3] + bias.squaredNorm());

    printf("b:[%f %f %f] x:[%4.1f %4.1f %4.1f] "
           "m:[%4.1f %4.1f %4.1f] B: %f\n",
           bias[0], bias[1], bias[2],
           b[0] - bias[0], b[1] - bias[1], b[2] - bias[2],
           b[0], b[1], b[2],
           bmag);

    timeval tv;
    gettimeofday(&tv, NULL);
    int delay = period - (tv.tv_usec % period);
    if (delay > 0)
      usleep(delay);
  }

  return 0;
}
