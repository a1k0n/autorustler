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

bool sirf_sendmsg(int fd, const uint8_t* msg, int msg_len) {
  static uint8_t header[2] = {0xa0, 0xa2},
                 footer[4] = {0, 0, 0xb0, 0xb3};
  static struct iovec iov[3] = {
    {header, 2}, {0, 0}, {footer, 4}};
  uint16_t cksum = 0;

  iov[1].iov_base = const_cast<uint8_t*>(msg);
  iov[1].iov_len = msg_len;

  for (int i = 0; i < msg_len; i++) {
    cksum += msg[i];
  }
  footer[0] = (cksum >> 8) & 0x7f;
  footer[1] = cksum & 255;
  int len = writev(fd, iov, 3);
  fprintf(stderr, "SiRF: writing ");
  for (int i = 0; i < msg_len; i++)
    fprintf(stderr, "%02x ", msg[i]);
  fprintf(stderr, "cksum %04x -> %02x %02x\n", cksum, footer[0], footer[1]);
  if (len == -1) {
    perror("SiRF: writev");
    return false;
  }
  return true;
}

static inline int32_t geti4(uint8_t *buf) {
  int32_t x = buf[0] << 24;
  x += buf[1] << 16;
  x += buf[2] << 8;
  x += buf[3];
  return x;
}

static inline int16_t geti2(uint8_t *buf) {
  int16_t x = buf[0] << 8;
  x += buf[1];
  return x;
}

void sirf_process_msg(uint8_t *buf, uint16_t len,
                      void (*navpos_cb)(const sirf_navdata& data)) {
  if (len == 0)
    return;
  switch (buf[0]) {
    case 0x02:
      {
        if (len != 41)
          goto badlength;
        struct sirf_navdata data;
        data.x = geti4(buf+1);
        data.y = geti4(buf+5);
        data.z = geti4(buf+9);
        data.v8x = geti2(buf+13);
        data.v8y = geti2(buf+15);
        data.v8z = geti2(buf+17);
        data.hdop = buf[20];
        data.svs = buf[28];
        data.gps_week = geti2(buf+22);
        data.gps_tow = geti4(buf+24);
        navpos_cb(data);
      }
      break;
    case 0x0b:
      fprintf(stderr, "SiRF: ACK message %02x\n", buf[1]);
      break;
    case 0x04:
    case 0x09:
    case 0x29:
    case 0x33:
    case 0x38:
    case 0x41:  // ???
    case 0x5c:  // ???
    case 0x5d:  // ???
      // ignore
      break;
    default:
      fprintf(stderr, "SiRF: unknown message ");
      for (int i = 0; i < len; i++) {
        fprintf(stderr, "%02x", buf[i]);
      }
      fprintf(stderr, "\n");
      break;
  }
  return;

 badlength:
  fprintf(stderr, "SiRF: msg %02x bad length %d\n", buf[0], len);
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
  sirf_sendmsg(fd, NULL, 0);

  usleep(10000);

  // set differential GPS source to SBAS satellite
  uint8_t dgps_config[] = {0x85, 0x01, 0, 0, 0, 0, 0};
  sirf_sendmsg(fd, dgps_config, sizeof(dgps_config));

  uint8_t poll_params[] = {0xa8, 0x85};
  sirf_sendmsg(fd, poll_params, sizeof(poll_params));

  return fd;
}

bool sirf_poll(int fd, void (*navpos_cb)(const sirf_navdata&)) {
  uint8_t buf[1024];
  static uint8_t msgbuf[1024];
  static int read_state = 0;
  static unsigned msg_length = 0, msg_ptr = 0;
  static int msg_cls = 0, msg_id = 0;
  static uint16_t msg_cksum = 0, input_cksum = 0;

  int len = read(fd, buf, sizeof(buf));
  int i;
  if (len == -1) {
    perror("read");
    return false;
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
          sirf_process_msg(msgbuf, msg_length, navpos_cb);
        }
        break;
    }
  }

  return true;
}
