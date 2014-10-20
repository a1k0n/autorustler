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
#include <vector>
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
const float pointcloud_dist = 200.0f;

bool CheckPointCloud(const Vector3f& pt, std::vector<Vector3f> *pointcloud) {
  for (int i = 0; i < pointcloud->size(); i++) {
    if (((*pointcloud)[i] - pt).squaredNorm() <
        pointcloud_dist*pointcloud_dist) {
      return false;
    }
  }
  fprintf(stderr, "added [%f %f %f] to point cloud #%d\n",
          pt[0], pt[1], pt[2], pointcloud->size());
  pointcloud->push_back(pt);
  return true;
}

int main() {
  int i2cfd = open("/dev/i2c-1", O_RDWR);
  if (i2cfd == -1) {
    perror("/dev/i2c-1");
    return 1;
  }

  signal(SIGINT, handle_sigint);

  IMU imu(i2cfd);

  const int period = 200000;

  Matrix4f XTX = 0.1 * Matrix4f::Identity();
  Vector4f XTY = Vector4f::Zero();
  std::vector<Vector3f> pointcloud;

  // our state is [nx ny nz bx by bz] where n is the unbiased magnetometer
  // reading, i.e., n = m - b, and b is the magnetometer bias bias.
  // prediction: n' = R(w) * n, b' = b, so F = [[R(w) 0] [0 I]]
  // measurement: z = n + b, so H = [I I]
  // Vector3f lastm = Vector3f::Zero();
  Vector4f beta = Vector4f::Zero();
  printf("# mx my mz bx by bz bmag\n");
  while (!done) {
    IMURawState s;
    timeval tv0;
    gettimeofday(&tv0, NULL);
    imu.ReadRaw(&s);

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
    Vector3f m(s.mag_x, s.mag_y, s.mag_z);
    float sampleweight = 1;
#if 0
    if (m.squaredNorm() > 0 && lastm.squaredNorm() > 0) {
      sampleweight = 1 - m.dot(lastm) / (lastm.norm() * m.norm());
    }
#else
    if (!CheckPointCloud(m, &pointcloud)) {
      Vector3f bias = beta.block<3, 1>(0, 0) / 2;
      float bmag = sqrt(beta[3] + bias.squaredNorm());
      Vector3f x = m - bias;
      fprintf(stderr, "b:[%f %f %f] x:[%4.1f %4.1f %4.1f] "
              "m:[%4.1f %4.1f %4.1f] B: %f\r",
              bias[0], bias[1], bias[2],
              x[0], x[1], x[2], m[0], m[1], m[2], bmag);
      fflush(stderr);
      continue;
    }
#endif
    XTX += sampleweight * b * b.transpose();
    // std::cerr << XTX << std::endl;
    float bmag = b.squaredNorm();
    XTY += sampleweight * bmag * b;
    // std::cerr << XTY.transpose() << std::endl;

    beta = XTX.ldlt().solve(XTY);
    Vector3f bias = beta.block<3, 1>(0, 0) / 2;
    bmag = sqrt(beta[3] + bias.squaredNorm());

    printf("%e %e %e %e %e %e %e\n",
           m[0], m[1], m[2], bias[0], bias[1], bias[2], bmag);
#if 0
    printf("b:[%f %f %f] x:[%4.1f %4.1f %4.1f] "
           "m:[%4.1f %4.1f %4.1f] B: %f\n",
           bias[0], bias[1], bias[2],
           b[0] - bias[0], b[1] - bias[1], b[2] - bias[2],
           b[0], b[1], b[2],
           bmag);
    lastm = m;
#endif

    timeval tv;
    gettimeofday(&tv, NULL);
    int delay = period - (tv.tv_usec % period);
    if (delay > 0)
      usleep(delay);
  }

  return 0;
}
