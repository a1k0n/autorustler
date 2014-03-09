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
    perror("I2C_RDWR");
    return 1;
  }

  return 0;
}


