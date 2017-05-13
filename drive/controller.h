#ifndef DRIVE_CONTROLLER_H_
#define DRIVE_CONTROLLER_H_

#include <Eigen/Dense>

class DriveController {
 public:
  DriveController() {
    last_side_ = 0;
  }

  void UpdateState(uint8_t *yuv, size_t yuvlen,
      uint16_t throttle_in, uint16_t steering_in,
      const Eigen::Vector3f &accel,
      const Eigen::Vector3f &gyro);

  bool GetControl(uint16_t *steer_out, uint16_t *throttle_out);

 private:
  // TODO: EKF for state estimation
  float alpha_, beta_;
  int n_, last_side_;
};

#endif  // DRIVE_CONTROLLER_H_
