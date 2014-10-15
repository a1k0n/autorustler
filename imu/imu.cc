#include <byteswap.h>
#include "gpio/i2c.h"
#include "imu/imu.h"

const uint8_t ADDR_ITG3200  = 0x68;
const uint8_t ADDR_HMC5883L = 0x1e;
const uint8_t ADDR_ADXL345  = 0x53;

// TODO(a1k0n): calibrate the compass.  there's a constant magnetic field added
// to earth's magnetic north, and we have to estimate and cancel it.  we may
// also need to compensate for the motor's magnetic fields when it's running?
//
// TODO(a1k0n): AGC for the compass.  you can adjust the gain, so we need to
// try to maximize the magnitude without overflowing the range.
//
// TODO(a1k0n): calibrate the gyro.  it also has a constant offset when nothing
// is moving, it seems.
//
// our motion model should figure out whether the car is on the ground (1g down
// on the accelerometer) or flying off a ramp (0g); if it's on the ground and
// the motor is stopped (0 estimated velocity) then we can zero out the gyro,
// and calibrate the direction of the accelerometer Y direction.

int imu_init(int i2cfd) {
  // config gyro
  i2c_write(i2cfd, ADDR_ITG3200, 0x3E, 0x01);  // use X gyro PLL oscillator
  i2c_write(i2cfd, ADDR_ITG3200, 0x15, 19);    // samplerate 50Hz (1000/(19+1))
  i2c_write(i2cfd, ADDR_ITG3200, 0x16, 0x18 + 4);  // enable, 20Hz bandwidth
  // config compass
  i2c_write(i2cfd, ADDR_HMC5883L, 0x00, 0x38);  // CRA: 75Hz rate w/ 2 averages
  i2c_write(i2cfd, ADDR_HMC5883L, 0x01, 0x20);  // CRB: set gain 1090 LSB/Gauss
  i2c_write(i2cfd, ADDR_HMC5883L, 0x02, 0x00);  // continuous measurement
  // config accelerometer
  i2c_write(i2cfd, ADDR_ADXL345, 0x2c, 0x09);  // 25Hz bw, 50Hz samplerate
  i2c_write(i2cfd, ADDR_ADXL345, 0x31, 0x08);  // FULL_RES
  i2c_write(i2cfd, ADDR_ADXL345, 0x38, 0x00);  // bypass FIFO, sample @ 50Hz
  i2c_write(i2cfd, ADDR_ADXL345, 0x2d, 0x08);  // turn on
  return 0;
}

int imu_read(int i2cfd, IMUState *s) {
  uint8_t axis_buf[8];
  i2c_read(i2cfd, ADDR_ITG3200, 0x1b, axis_buf, 8);
  // temperature is 280 LSB/deg C, -13200 LSB @35 C
  s->gyro_temp = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+0));
  s->gyro_x = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+2));  // roll
  s->gyro_y = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+4));  // pitch
  s->gyro_z = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+6));  // yaw

  i2c_read(i2cfd, ADDR_HMC5883L, 0x03, axis_buf, 6);
  s->mag_x = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+0));  // front?
  s->mag_y = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+2));  // up
  s->mag_z = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+4));  // side?

  i2c_read(i2cfd, ADDR_ADXL345, 0x32, axis_buf, 6);
  s->accel_x = (*reinterpret_cast<uint16_t*>(axis_buf+0));  // toward back
  s->accel_y = (*reinterpret_cast<uint16_t*>(axis_buf+2));  // toward right
  s->accel_z = (*reinterpret_cast<uint16_t*>(axis_buf+4));  // toward ground
  return 0;
}
