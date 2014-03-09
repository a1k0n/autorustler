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



int main() {
  int i2cfd = open("/dev/i2c-1", O_RDWR);
  if (i2cfd == -1) {
    perror("/dev/i2c-1");
    return 1;
  }

  // config gyro: 10Hz bandwidth
  i2c_write(i2cfd, ADDR_ITG3200, 0x16, 0x18 + 5);
  // config compass: continuous measurement
  i2c_write(i2cfd, ADDR_HMC5883L, 0x02, 0x00);
  for (;;) {
    uint8_t axis_buf[6];
    int16_t gyro_x, gyro_y, gyro_z,
            mag_x, mag_y, mag_z;
    i2c_read(i2cfd, ADDR_ITG3200, 0x1d, axis_buf, 6);
    gyro_x = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+0));
    gyro_y = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+2));
    gyro_z = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+4));
    i2c_read(i2cfd, ADDR_HMC5883L, 0x03, axis_buf, 6);
    mag_x = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+0));
    mag_y = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+2));
    mag_z = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+4));
    fprintf(stderr, "gyro [%+4d %+4d %+4d] mag [%+4d %+4d %+4d]\e[K\r",
            gyro_x, gyro_y, gyro_z,
            mag_x, mag_y, mag_z);
    fflush(stderr);
    usleep(200000);
  }

  return 0;
}
