#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <iostream>

#include "drive/controller.h"
#include "drive/imgproc.h"

using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::MatrixXf;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::VectorXf;

static const float MAX_THROTTLE = 1.0;
static const float SPEED_LIMIT = 3.4;

static const float ACCEL_LIMIT = 8.0;  // maximum dv/dt (m/s^2)
static const float BRAKE_LIMIT = -100.0;  // minimum dv/dt
static const float TRACTION_LIMIT = 3.0;  // maximum v*w product (m/s^2)
static const float kpy = 0.2;
static const float kvy = 1.0;

static const float LANE_OFFSET = 0.8;
// this is actually used in model.py, not here
static const float METERS_PER_ENCODER_TICK = M_PI * 0.101 / 20;


DriveController::DriveController() {
  ResetState();
}

void DriveController::ResetState() {
  ekf.Reset();
  firstframe_ = true;
}

static inline float clip(float x, float min, float max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

void DriveController::UpdateCamera(const uint8_t *yuv) {
  Vector3f B;
  Matrix3f Rk;

  if (!TophatFilter(yuv, &B, &Rk)) {
    return;
  }
  // Rk(1, 1) += 0.01;  // slope is a bit iffy; add a bit of noise covariance

  // ok, we've obtained our linear fit B and our measurement covariance Rk
  // now do the sensor fusion step

  float a = B[0], b = B[1], c = B[2];
  printf("tophat found abc %f %f %f implied ye %f psie %f k %f\n", a, b, c,
      -c, atan(b), 2*a*pow(b*b + 1, -1.5));

  ekf.UpdateCenterline(a, b, c, Rk);
}

void DriveController::UpdateState(const uint8_t *yuv, size_t yuvlen,
      float throttle_in, float steering_in,
      const Vector3f &accel, const Vector3f &gyro,
      uint8_t servo_pos, const uint16_t *wheel_encoders, float dt) {
  Eigen::VectorXf &x_ = ekf.GetState();
  if (isinf(x_[0]) || isnan(x_[0])) {
    fprintf(stderr, "WARNING: kalman filter diverged to inf/NaN! resetting!\n");
    ResetState();
    // exit(1);
    return;
  }

  if (firstframe_) {
    memcpy(last_encoders_, wheel_encoders, 4*sizeof(uint16_t));
    firstframe_ = false;
  }

  ekf.Predict(1.0/30.0, throttle_in, steering_in);
  std::cout << "x after predict " << x_.transpose() << std::endl;

#if 1
  if (yuvlen == 640*480 + 320*240*2) {
    UpdateCamera(yuv);
    std::cout << "x after camera " << x_.transpose() << std::endl;
  } else {
    fprintf(stderr, "DriveController::UpdateState: invalid yuvlen %ld\n",
        yuvlen);
  }
#endif

  // ekf.UpdateIMU(gyro[2]);
  // std::cout << "x after IMU (" << gyro[2] << ")" << x_.transpose() << std::endl;

  // hack: force psi_e forward-facing
  if (x_[3] > M_PI/2) {
    x_[3] -= M_PI;
  } else if (x_[3] < -M_PI/2) {
    x_[3] += M_PI;
  }

  // read / update servo & encoders
  // use the average of the two rear encoders as we're most interested in the
  // motor speed
  // but we could use all four to get turning radius, etc.
  // since the encoders are just 16-bit counters which wrap frequently, we only
  // track the difference in counts between updates.
  printf("encoders were: %05d %05d %05d %05d\n"
         "      are now: %05d %05d %05d %05d\n",
      last_encoders_[0], last_encoders_[1], last_encoders_[2], last_encoders_[3],
      wheel_encoders[0], wheel_encoders[1], wheel_encoders[2], wheel_encoders[3]);

  // average ds among wheel encoders which are actually moving
  float ds = 0, nds = 0;
  for (int i = 0; i < 4; i++) {
    if (wheel_encoders[i] != last_encoders_[i]) {
      ds += (uint16_t) (wheel_encoders[2] - last_encoders_[2]);
      nds += 1;
    }
  }
  memcpy(last_encoders_, wheel_encoders, 4*sizeof(uint16_t));
  // and do an EKF update if the wheels are moving.
  if (nds > 0) {
    ekf.UpdateEncoders(ds/(nds * dt), servo_pos);
    std::cout << "x after encoders (" << ds/dt << ") " << x_.transpose() << std::endl;
  }

  std::cout << "P " << ekf.GetCovariance().diagonal().transpose() << std::endl;
}

static float MotorControl(float accel,
    float k1, float k2, float k3, float k4,
    float v) {
  float a_thresh = -k3 * v - k4;
  // voltage (1 or 0)
  float V = accel > a_thresh ? 1 : 0;
  // duty cycle
  float DC = clip(
      (accel + k3*v + k4) / (V*k1 - k2*v),
      0, 1);
  return V == 1 ? DC : -DC;
}

bool DriveController::GetControl(float *throttle_out, float *steering_out) {
  const Eigen::VectorXf &x_ = ekf.GetState();
  float v = x_[0];
  float delta = x_[1];
  float y_e = x_[2];
  float psi_e = x_[3];
  float kappa = x_[4];
#if 1
  float ml_1 = x_[5];
  float ml_2 = x_[6];
  float ml_3 = x_[7];
  float ml_4 = x_[8];
  float srv_a = x_[9];
  float srv_b = x_[10];
  float srv_r = x_[11];

  float k1 = exp(ml_1), k2 = exp(ml_2), k3 = exp(ml_3), k4 = exp(ml_4);
#else
  float srv_a = x_[5];
  float srv_b = x_[6];
  float srv_r = x_[7];
  float k1 = 5000 * M_PI * 0.101 / 40.0;
  float k2 = 4, k3 = 1;
  float k4 = 0.3 * k1;
#endif


  float dt = 1.0f/30;  // FIXME, should be an argument

  float vmax = fmin(SPEED_LIMIT, (k1 - k4)/(k2 + k3));

  // TODO: race line following w/ particle filter localization
  float lane_offset = LANE_OFFSET;
  float psi_offset = 0;

  float cpsi = cos(psi_e - psi_offset),
        spsi = sin(psi_e - psi_offset);
  float dx = cpsi / (1.0 - kappa*y_e);

  // Alain Micaelli, Claude Samson. Trajectory tracking for unicycle-type and
  // two-steering-wheels mobile robots. [Research Report] RR-2097, INRIA. 1993.
  // <inria-00074575>

  // it's a little backwards though because our steering is reversed w.r.t. curvature
  float k_target = -dx * (-(y_e - lane_offset) * dx * kpy*cpsi - spsi*(-kappa*spsi - kvy*cpsi) + kappa);
  float v_target = fmin(vmax, sqrtf(TRACTION_LIMIT / fabs(k_target)));
  float a_target = clip((v_target - v)/(4*dt), BRAKE_LIMIT, ACCEL_LIMIT);

  printf("steer_target %f delta %f v_target %f v %f a_target %f lateral_a %f/%f v %f y %f psi %f\n",
      k_target, delta, v_target, v, a_target, v*v*delta, TRACTION_LIMIT, v, y_e, psi_e);

  *steering_out = clip((k_target - srv_b) / srv_a, -1, 1);
  *throttle_out = clip(
      MotorControl(a_target, k1, k2, k3, k4, v),
      -1, MAX_THROTTLE);

  printf("  throttle %f steer %f\n", *throttle_out, *steering_out);
  return true;
}

