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

using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::MatrixXf;

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

// covariance of magnetometer reading / state
const float kMagStateCov = 1.0;
const float kMagOffsetCov = 0.0;
const float kMagReadingCov = 16.0;

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

  Vector3f lastx(s.mag_x, s.mag_y, s.mag_z);
  // http://www.roboticsproceedings.org/rss09/p50.pdf
  const float h = period / 1000000.0f;
  Matrix6f Q = Matrix6f::Zero();
  Q.topLeftCorner(3, 3) = Matrix3f::Identity() * kMagStateCov;
  Q.bottomRightCorner(3, 3) = Matrix3f::Identity() * kMagOffsetCov;
  Matrix3f R = Matrix3f::Identity() * kMagReadingCov;

  // state covariance estimate, initially large
  Matrix6f P = 10 * Matrix6f::Identity();
  Vector6f xk = Vector6f::Zero();  // predicted state, initially 0 also
  Vector3f wprev = Vector3f::Zero();
  Vector3f mprev = Vector3f::Zero();

  while (!done) {
    timeval tv0;
    gettimeofday(&tv0, NULL);
    imu_read(i2cfd, &s);

    // angular velocity in car coords
    // car: x right, y up, z forward
    // gyro: x roll (z), y -pitch(-x), z -yaw (-y)
    Vector3f w0(-s.gyro_y, -s.gyro_z, s.gyro_x);
    // scale to radians/second
    w0 -= gyrocal_b + s.gyro_temp * gyrocal_m;
    // datasheet says scale factor is 14.375 LSB/deg/s
    Vector3f w = w0 * M_PI / (180.0 * 14.375);
    // mag: x back (-z), y up(y), z right
    Vector3f m(s.mag_z, s.mag_y, -s.mag_x);
    // accel: x front (z), y left (-x), z down (-y)
    Vector3f g(-s.accel_y, -s.accel_z, s.accel_x);

    // integrate using the midpoint method
    Vector3f wmid = (wprev + w) / 2;

    // Kalman filter: F matrix depends on w, so recompute it here
    Matrix6f F = Matrix6f::Zero();
    float wnorm = wmid.norm();
    Matrix3f Rw = Matrix3f::Identity();
    if (wnorm > 0) {
      // if we rotate to the right, then the apparent rotation of the north
      // pole should be left, so the angle is -wnorm * h
      Rw = Eigen::AngleAxisf(-wnorm * h, wmid / wnorm);
    }
    F.topLeftCorner(3, 3) = Rw;
    F.topRightCorner(3, 3) = Matrix3f::Identity() - Rw;
    F.bottomRightCorner(3, 3) = Matrix3f::Identity();
    F.eval();

    // std::cout << "---- F ----" << std::endl;
    // std::cout << F << std::endl;

    // the H matrix is a 3x6 matrix with just identity in the left half
    // and will be used implicitly w/ topLeftCorner(3, 3)

    // predicted new state & covariance
    xk = F * xk;
    P = F * P * F.transpose() + Q;
#if 0
    Vector3f m1 = R * mprev;
    printf("prev: [%f %f %f] pred: [%f %f %f] actual: [%f %f %f]\n",
           mprev[0], mprev[1], mprev[2],
           m1[0], m1[1], m1[2], m[0], m[1], m[2]);
#endif

    Vector3f y = m - xk.topLeftCorner(3, 1);
    Matrix3f S = P.topLeftCorner(3, 3) + R;
    // X is the inverse of S, transposed... but S is symmetric anyway
    Matrix3f X = S.ldlt().solve(Matrix3f::Identity());
    MatrixXf K = P.topLeftCorner(6, 3) * X; // K is 6x3
    std::cout << "---- K ----" << std::endl;
    std::cout << K << std::endl;

    // compute new estimated state and covariance
    xk += K * y;
    P.topLeftCorner(6, 3) -= K;

    printf("%0.1f C b:[%f %f %f] w:[%5.2f %5.2f %5.2f] x:[%4.1f %4.1f %4.1f] "
           "y:[%4.1f %4.1f %4.1f]\n",
           (s.gyro_temp - -13200) * (1.0 / 280.0) + 35.0,
           xk[3], xk[4], xk[5],
           w[0], w[1], w[2],
           xk[0] - xk[3], xk[1] - xk[4], xk[2] - xk[5],
           y[0], y[1], y[2]);
    std::cout << "---- P ----" << std::endl;
    std::cout << P << std::endl;

    wprev = w;
    mprev = m;

#if 0
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
#endif

    timeval tv;
    gettimeofday(&tv, NULL);
    int delay = period - (tv.tv_usec % period);
    if (delay > 0)
      usleep(delay);
  }

  return 0;
}
