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
      const Eigen::Vector3f &gyro,
      uint8_t servo_pos,
      const uint16_t *wheel_encoders, float dt);

  bool GetControl(float *throttle_out, float *steering_out);

  void ResetState();

  // run a prediction step
  void PredictStep(float u_acceleration, float u_steering,
      float dt = 1.0/30);

  // update w/ slope and intercept of line on ground
  void UpdateCamera(const uint8_t *yuv);

  // update w/ IMU measurement
  void UpdateIMU(const Eigen::Vector3f &accel, const Eigen::Vector3f &gyro,
      float u_acceleration);
  void UpdateServoAndEncoders(float servo_pos, float ds);

  // state is:
  // ye, psie, w, v, k, Cv, Tv, Cs, Ts, mu_s, mu_g, mu_ax, mu_ay
  Eigen::VectorXf x_;
  Eigen::MatrixXf P_;

  bool firstframe_;
  uint16_t last_encoders_[4];
};

#endif  // DRIVE_CONTROLLER_H_
