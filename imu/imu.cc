#include <byteswap.h>
#include "./i2c.h"
#include "./imu.h"

const uint8_t ADDR_ITG3200  = 0x68;
const uint8_t ADDR_HMC5883L = 0x1e;
const uint8_t ADDR_ADXL345  = 0x53;

// TODO: how do i set the i2c clock rate?
// we should be able to crank it up to 400kHz


// TODO: calibrate the compass.  there's a constant magnetic field added to
// earth's magnetic north, and we have to estimate and cancel it.
// we may also need to compensate for the motor's magnetic fields when it's
// running?
// TODO: AGC for the compass.  you can adjust the gain, so we need to try to
// maximize the magnitude without overflowing the range.
// TODO: calibrate the gyro.  it also has a constant offset when nothing is
// moving, it seems.
//
// our motion model should figure out whether the car is on the ground (1g down
// on the accelerometer) or flying off a ramp (0g); if it's on the ground and
// the motor is stopped (0 estimated velocity) then we can zero out the gyro,
// and calibrate the direction of the accelerometer Y direction.

int imu_init(int i2cfd) {
  // config gyro
  i2c_write(i2cfd, ADDR_ITG3200, 0x16, 0x18 + 2);  // enable, 100Hz bandwidth
  // config compass
  i2c_write(i2cfd, ADDR_HMC5883L, 0x02, 0x00);  // continuous measurement
  i2c_write(i2cfd, ADDR_HMC5883L, 0x01, 0x20);  // set gain
  // config accelerometer
  i2c_write(i2cfd, ADDR_ADXL345, 0x2d, 0x08);  // turn on
  i2c_write(i2cfd, ADDR_ADXL345, 0x31, 0x08);  // FULL_RES
  return 0;
}

int imu_read(int i2cfd, imu_state *s) {
  uint8_t axis_buf[6];
  i2c_read(i2cfd, ADDR_ITG3200, 0x1d, axis_buf, 6);
  s->gyro_x = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+0));  // roll
  s->gyro_y = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+2));  // pitch
  s->gyro_z = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+4));  // yaw

  i2c_read(i2cfd, ADDR_HMC5883L, 0x03, axis_buf, 6);
  // facing south-ish: +270, -430, -67
  // facing north-ish: +420, -230, +60
  // facing east-ish: +300, -300, +53
  // facing west-ish: +500, -36, +60
  // facing up, top towards north: -288, +410, -50
  // facing up, top towards east: -300, +300, -130
  // up, towards west: -280, +275, +110
  // on right side, top toward west: 130, 300, -630
  // this is confounded by a metal bench, not sure what's going on
  s->mag_x = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+0));  // front?
  s->mag_y = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+2));  // up
  s->mag_z = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+4));  // side?

  i2c_read(i2cfd, ADDR_ADXL345, 0x32, axis_buf, 6);
  // note: not totally calibrated across the three axes; very small offset
  // error but the z axis seems to have its scale off a bit -- -270 to +245
  s->accel_x = (*reinterpret_cast<uint16_t*>(axis_buf+0));  // toward back
  s->accel_y = (*reinterpret_cast<uint16_t*>(axis_buf+2));  // toward right
  s->accel_z = (*reinterpret_cast<uint16_t*>(axis_buf+4));  // toward ground
  return 0;
}
