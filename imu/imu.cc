#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <byteswap.h>
#include <unistd.h>
#include "./i2c.h"

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
//

int main() {
  int i2cfd = open("/dev/i2c-1", O_RDWR);
  if (i2cfd == -1) {
    perror("/dev/i2c-1");
    return 1;
  }

  // config gyro
  i2c_write(i2cfd, ADDR_ITG3200, 0x16, 0x18 + 2);  // enable, 100Hz bandwidth
  // config compass
  i2c_write(i2cfd, ADDR_HMC5883L, 0x02, 0x00);  // continuous measurement
  i2c_write(i2cfd, ADDR_HMC5883L, 0x01, 0x20);  // set gain
  // config accelerometer
  i2c_write(i2cfd, ADDR_ADXL345, 0x2d, 0x08);  // turn on
  i2c_write(i2cfd, ADDR_ADXL345, 0x31, 0x08);  // FULL_RES
  for (;;) {
    uint8_t axis_buf[6];
    int16_t gyro_x, gyro_y, gyro_z,
            mag_x, mag_y, mag_z,
            accel_x, accel_y, accel_z;
    i2c_read(i2cfd, ADDR_ITG3200, 0x1d, axis_buf, 6);
    gyro_x = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+0));  // roll
    gyro_y = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+2));  // pitch
    gyro_z = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+4));  // yaw

    i2c_read(i2cfd, ADDR_HMC5883L, 0x03, axis_buf, 6);
    mag_x = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+0));  // front/side?
    mag_y = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+2));  // up
    mag_z = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+4));  // side/front?

    i2c_read(i2cfd, ADDR_ADXL345, 0x32, axis_buf, 6);
    accel_x = (*reinterpret_cast<uint16_t*>(axis_buf+0));
    accel_y = (*reinterpret_cast<uint16_t*>(axis_buf+2));
    accel_z = (*reinterpret_cast<uint16_t*>(axis_buf+4));

    fprintf(stderr, "gyro [%+4d %+4d %+4d] mag [%+4d %+4d %+4d] "
            "acc [%+4d %+4d %+4d]\e[K\r",
            gyro_x, gyro_y, gyro_z,
            mag_x, mag_y, mag_z,
            accel_x, accel_y, accel_z);
    fflush(stderr);
    usleep(100000);
  }

  return 0;
}
