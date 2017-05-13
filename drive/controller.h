#ifndef DRIVE_CONTROLLER_H_
#define DRIVE_CONTROLLER_H_

#include <Eigen/Dense>
#include <math.h>

class DriveController {
 public:
  DriveController();

  // do full kalman filter update: prediction and sensor fusion
  void UpdateState(const uint8_t *yuv, size_t yuvlen,
      float throttle_in, float steering_in,
      const Eigen::Vector3f &accel,
      const Eigen::Vector3f &gyro, float dt = 1.0/30.0);

  bool GetControl(uint16_t *steer_out, uint16_t *throttle_out);

  void ResetState();

  // run a prediction step
  void PredictStep(float u_acceleration, float u_steering,
      float dt = 1.0/30);

  // update w/ slope and intercept of line on ground
  void UpdateCamera(const uint8_t *yuv);

  // update w/ IMU measurement
  void UpdateIMU(const Eigen::Vector3f &accel, const Eigen::Vector3f &gyro,
      float u_acceleration);

  // state is:
  // ye, psie, w, v, k, Cv, Tv, Cs, Ts, mu_s, mu_g, mu_ax, mu_ay
  Eigen::VectorXf x_;
  Eigen::MatrixXf P_;
};

#endif  // DRIVE_CONTROLLER_H_
