#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include "./i2c.h"

int i2c_read(int file, uint8_t addr, uint8_t reg,
             uint8_t *outbuf, int len) {
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[2];

  messages[0].addr  = addr;
  messages[0].flags = 0;
  messages[0].len   = 1;
  messages[0].buf   = &reg;

  messages[1].addr  = addr;
  messages[1].flags = I2C_M_RD;
  messages[1].len   = len;
  messages[1].buf   = outbuf;

  packets.msgs      = messages;
  packets.nmsgs     = 2;
  if (ioctl(file, I2C_RDWR, &packets) < 0) {
    perror("i2c_read");
    return 1;
  }

  return 0;
}

int i2c_write(int file, uint8_t addr, uint8_t reg, uint8_t value) {
  uint8_t outbuf[2];
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[1];

  outbuf[0] = reg;
  outbuf[1] = value;

  messages[0].addr  = addr;
  messages[0].flags = 0;
  messages[0].len   = 2;
  messages[0].buf   = outbuf;

  packets.msgs  = messages;
  packets.nmsgs = 1;
  if (ioctl(file, I2C_RDWR, &packets) < 0) {
    perror("i2c_write");
    return 1;
  }
  return 0;
}
