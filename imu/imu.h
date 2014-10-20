#ifndef IMU_IMU_H_
#define IMU_IMU_H_

#include <stdint.h>
#include <vector>
#include <Eigen/Dense>

// TODO: rename to imu/dev.h for IMU device raw access
// then make an IMU class which is self-calibrating, has a kalman filter, etc.

struct IMURawState {
  int16_t gyro_x, gyro_y, gyro_z, gyro_temp;
  int16_t mag_x, mag_y, mag_z;
  int16_t accel_x, accel_y, accel_z;
};

struct IMUState {
  Eigen::Vector3f w;  // angular velocity, rads/sec (gyro)
  Eigen::Vector3f N;  // north pole direction (mag)
  Eigen::Vector3f g;  // acceleration + gravity (m/s^2)
};

class IMU {
 public:
  IMU();
  explicit IMU(int i2cfd);
  ~IMU();

  bool Init(int i2cfd);

  // read raw sensor values
  bool ReadRaw(IMURawState *state);

  // Read adjusted sensor values after various calibrations (gyro
  // temperature-compensated offset, magnetometer offset, accelerometer offset)
  // and adjusting axes to the car's local frame (x right, y up, z forward)
  bool ReadCalibrated(IMUState *state);

  // convert raw readings to calibrated ones; ReadCalibrated just calls this
  void Calibrate(const IMURawState &rawstate, IMUState *state);

 private:
  int fd_;

  std::vector<Eigen::Vector3f> mag_cal_points_;
  Eigen::Matrix4f mag_XTX_;
  Eigen::Vector4f mag_XTY_;
  Eigen::Vector3f mag_bias_;
};

#endif  // IMU_IMU_H_
