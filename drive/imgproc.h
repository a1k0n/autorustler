#ifndef DRIVE_IMGPROC_H_
#define DRIVE_IMGPROC_H_

#include <Eigen/Dense>

bool TophatFilter(const uint8_t *yuv, Eigen::Vector3f *Bout, Eigen::Matrix3f *Rkout);

#endif  // DRIVE_IMGPROC_H_
