#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <termios.h>
#include <unistd.h>

#include "./sirf.h"

const char sirf_port[] = "/dev/ttyAMA0";
#define startup_ioctl_baud B4800
#define runtime_baudrate 38400
#define runtime_ioctl_baud B38400

namespace {

void nmea_sendmsg(int fd, const char *msg) {
  int i, cksum = 0, len = strlen(msg);
  char footer[6];
  for (i = 1; i < len; i++)
    cksum ^= msg[i];
  snprintf(footer, sizeof(footer), "*%02x\r\n", cksum);
  write(fd, msg, len);
  write(fd, footer, 5);
}

}  // empty namespace

int sirf_open() {
  int fd = open(sirf_port, O_RDWR);
  if (fd == -1) {
    perror(sirf_port);
    return -1;
  }

  struct termios tios;
  tcgetattr(fd, &tios);
  // disable flow control and all that, and ignore break and parity errors
  tios.c_iflag = IGNBRK | IGNPAR;
  tios.c_oflag = 0;
  tios.c_lflag = 0;
  cfsetspeed(&tios, startup_ioctl_baud);
  tcsetattr(fd, TCSAFLUSH, &tios);
  // the serial port has a brief glitch once we turn it on which generates a
  // start bit; sleep for 1ms to let it settle
  usleep(10000);

  write(fd, "\r\n\r\n", 4);

  // send baudrate/protocol change message
  char nmeamsg[256];
  snprintf(nmeamsg, sizeof(nmeamsg),
           "$PSRF100,0,%d,8,1,0", runtime_baudrate);
  nmea_sendmsg(fd, nmeamsg);

  // add some guard time before and after changing baud rates
  usleep(100000);
  close(fd);
  fd = open(sirf_port, O_RDWR);
  cfsetspeed(&tios, runtime_ioctl_baud);
  tcsetattr(fd, TCSADRAIN, &tios);

  if (fd == -1) {
    perror(sirf_port);
    return -1;
  }

  // FIXME: sirf_sendmsg
  // FIXME: sirf_enable_periodic
  return fd;
}

void sirf_process_msg(uint8_t *buf, uint16_t len) {
  for (int i = 0; i < len; i++) {
    printf("%02x", buf[i]);
  }
  printf("\n");
}

void sirf_read_loop(int fd) {
  uint8_t buf[1024];
  static uint8_t msgbuf[1024];
  static int read_state = 0;
  static unsigned msg_length = 0, msg_ptr = 0;
  static int msg_cls = 0, msg_id = 0;
  static uint16_t msg_cksum = 0, input_cksum = 0;
  for (;;) {
    int len = read(fd, buf, sizeof(buf));
    int i;
    if (len == -1) {
      perror("read");
      return;
    }
    for (i = 0; i < len; i++) {
      // printf("%02x ", buf[i]);
      switch (read_state) {
        case 0:
          if (buf[i] == 0xa0) read_state++;
          break;
        case 1:
          read_state++;
          if (buf[i] != 0xa2) read_state = 0;
          break;
        case 2:
          msg_length = buf[i] << 8;
          read_state++;
          break;
        case 3:
          msg_length += buf[i];
          msg_ptr = 0;
          msg_cksum = 0;
          if (msg_length > sizeof(msgbuf)) {
            fprintf(stderr, "SiRF: discarding message length %d\n",
                    msg_length);
            read_state = 0;
          } else {
            read_state++;
          }
          break;
        case 4:
          msgbuf[msg_ptr++] = buf[i];
          msg_cksum += buf[i];
          if (msg_ptr == msg_length)
            read_state++;
          break;
        case 5:
          input_cksum = buf[i] << 8;
          read_state++;
          break;
        case 6:
          input_cksum += buf[i];
          read_state++;
          if (input_cksum != (msg_cksum & 0x7fff)) {
            fprintf(stderr, "SiRF: checksum mismatch, calc %04x got %04x\n",
                    msg_cksum & 0x7fff, input_cksum);
            read_state = 0;
          }
          break;
        case 7:
          read_state++;
          if (buf[i] != 0xb0) {
            fprintf(stderr, "SiRF: missing 0xb0 terminator\n");
            read_state = 0;
          }
          break;
        case 8:
          read_state = 0;
          if (buf[i] != 0xb3) {
            fprintf(stderr, "SiRD: missing 0xb3 terminator\n");
          } else {
            sirf_process_msg(msgbuf, msg_length);
          }
          break;
      }
    }
    // printf("\n");
  }
}
