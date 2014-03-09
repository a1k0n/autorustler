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

  for (;;) {
    uint8_t gyro_buf[6];
    uint16_t gyro_x, gyro_y, gyro_z;
    i2c_read(i2cfd, ADDR_ITG3200, 0x1d, gyro_buf, 6);
    gyro_x = bswap_16(*reinterpret_cast<uint16_t*>(gyro_buf+0));
    gyro_y = bswap_16(*reinterpret_cast<uint16_t*>(gyro_buf+2));
    gyro_z = bswap_16(*reinterpret_cast<uint16_t*>(gyro_buf+4));
    fprintf(stderr, "%04x %04x %04x\r", gyro_x, gyro_y, gyro_z);
    fflush(stderr);
    usleep(200000);
  }

  return 0;
}
