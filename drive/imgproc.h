#ifndef DRIVE_IMGPROC_H_
#define DRIVE_IMGPROC_H_

#include <Eigen/Dense>

bool TophatFilter(const uint8_t *yuv, Eigen::Vector3f *Bout,
    float *y_cout, Eigen::Matrix4f *Rkout);

#endif  // DRIVE_IMGPROC_H_
