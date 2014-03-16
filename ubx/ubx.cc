#include <math.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <termios.h>
#include <unistd.h>

#include "vec3.h"

const char ubx_port[] = "/dev/ttyAMA0";
#define startup_ioctl_baud B9600
#define runtime_baudrate 115200
#define runtime_ioctl_baud B115200

void nmea_sendmsg(int fd, const char *msg) {
  int i, cksum = 0, len = strlen(msg);
  char footer[6];
  for (i = 1; i < len; i++)
    cksum ^= msg[i];
  sprintf(footer, "*%02x\r\n", cksum);
  write(fd, msg, len);
  write(fd, footer, 5);
}

void ubx_sendmsg(int fd, int msg_class, int msg_id,
                 const uint8_t *msg, int msg_len) {
  static uint8_t header[6] = {0xb5, 0x62}, footer[2];
  static struct iovec iov[3] = {
    {header, 6}, {0, 0}, {footer, 2}};
  uint8_t cka, ckb;
  int i;
  iov[1].iov_base = (void*) msg;
  iov[1].iov_len = msg_len;
  header[2] = msg_class;
  cka = ckb = msg_class;
  header[3] = msg_id;
  cka += msg_id; ckb += cka;
  header[4] = msg_len & 0xff;
  cka += header[4]; ckb += cka;
  header[5] = msg_len >> 8;
  cka += header[5]; ckb += cka;
  for (i = 0; i < msg_len; i++) {
    cka += msg[i]; ckb += cka;
  }
  footer[0] = cka;
  footer[1] = ckb;
  int len = writev(fd, iov, 3);
#if 0
  for (i = 0; i < 6; i++)
    printf("%02x ", header[i]);
  printf("| ");
  for (i = 0; i < msg_len; i++)
    printf("%02x ", msg[i]);
  printf("| ");
  printf("%02x %02x -- wrote %d\n", footer[0], footer[1], len);
#endif
  if (len == -1) {
    perror("ubx_sendmsg: writev");
  }
}

void ubx_enable_periodic(int fd, int msgclass, int msgid, int enable) {
  printf("%sabling (%d,%d)\n", enable ? "en" : "dis", msgclass, msgid);
  // now, CFG-MSG and set NAV-SOL output on every epoch
  uint8_t cfg_msg[] = {
    msgclass, msgid, // class/id of NAV-POSLLH
    0, enable, 0, 0, 0, 0  // output once per epoch on port 1
  };
  ubx_sendmsg(fd, 6, 1, cfg_msg, 8);
}

// Earth-Centered, Earth-Fixed position message from GPS
struct nav_posecef {
  uint32_t iTOW;  // millisecond time of week
  int32_t ecefX, ecefY, ecefZ; // ECEF position in cm
  uint32_t pAcc;  // position accuracy estimate
};

bool have_reference = false;
// reference position, taken from first measurement
vec3<uint32_t> ref_ecef(-1, -1, -1);
// orthonormal basis for local reference plane
vec3<double> ref_up, ref_north, ref_east;

// mean and precision (= 1/variance) of current north/east estimate
vec2<float> mean_pos;
float prec_ne;  // precision (cm) in local reference plane
// eventually precision will be a 2x2 matrix once we have a heading and a
// predictive model, or just a bunch of particles

void compute_refplane() {
  static const double wgs84_inverse_flattening = 298.257223563;
  static const double wgs84_stretching = 1.0 / (wgs84_inverse_flattening - 1);
  ref_up = vec3<double>(ref_ecef.x, ref_ecef.y, ref_ecef.z);
  ref_up.z += ref_up.z * wgs84_stretching;
  ref_up.normalize();
  ref_north = (vec3<double>(0,0,1) - ref_up * ref_up.z).normalize();
  ref_east = cross(ref_north, ref_up);
}

void process_msg(int fd, int msg_class, int msg_id,
                 uint8_t *msgbuf, int msg_length) {
  int i;
  switch ((msg_class << 8) + msg_id) {
    case 0x0101:  // NAV-POSECEF
      {
        struct nav_posecef *navmsg = (struct nav_posecef*) msgbuf;
        printf("POS-ECEF @%10d xyz (%d, %d, %d) +- %d\n", navmsg->iTOW,
               navmsg->ecefX, navmsg->ecefY, navmsg->ecefZ, navmsg->pAcc);
        if (!have_reference) {
          ref_ecef = vec3<uint32_t>(
              navmsg->ecefX, navmsg->ecefY, navmsg->ecefZ);
          have_reference = true;
          compute_refplane();
        } else {
          vec3<int32_t> relpos(
              navmsg->ecefX - ref_ecef.x,
              navmsg->ecefY - ref_ecef.y,
              navmsg->ecefZ - ref_ecef.z);
          vec3<double> local_pos(
              ref_east * relpos,
              ref_north * relpos,
              ref_up * relpos);
          printf("(%0.0fN, %0.0fE, %0.0fU)\n",
                 local_pos.x, local_pos.y, local_pos.z);
        }
      }
      break;
    case 0x0102:  // NAV-POSLLH (ignored)
      break;
    case 0x0501:  // ACK
    case 0x0500:  // NAK
      printf("%s (%d,%d)\n", msg_id == 1 ? "ack" : "nak",
             msgbuf[0], msgbuf[1]);
      break;
    default:
      printf("read msg (%02x,%02x): ", msg_class, msg_id);
      for (i = 0; i < msg_length; i++)
        printf("%02x ", msgbuf[i]);
      printf("\n");
      // diable unknown messages?
      // ubx_enable_periodic(fd, msg_class, msg_id, 0);
  }
}

void read_loop(int fd) {
  uint8_t buf[512];
  static uint8_t msgbuf[512];
  static int read_state = 0;
  static unsigned msg_length = 0, msg_ptr = 0;
  static int msg_cls = 0, msg_id = 0;
  static uint8_t msg_cka = 0, msg_ckb = 0;
  for(;;) {
    int len = read(fd, buf, sizeof(buf));
    int i;
    if (len == -1) {
      perror("read");
      return;
    }
    for(i = 0; i < len; i++) {
      // printf("%02x ", buf[i]);
      if (read_state > 1 && read_state < 7) {
        msg_cka += buf[i];
        msg_ckb += msg_cka;
      }
      switch (read_state) {
        case 0:
          if (buf[i] == 0xb5) read_state++;
          else { putchar(buf[i]); fflush(stdout); }
          break;
        case 1:
          if (buf[i] == 0x62) read_state++;
          else if (buf[i] != 0xb5) read_state = 0;
          msg_cka = 0; msg_ckb = 0;
          break;
        case 2:
          msg_cls = buf[i]; read_state++;
          break;
        case 3:
          msg_id = buf[i]; read_state++;
          break;
        case 4:
          msg_length = buf[i]; read_state++;
          break;
        case 5:
          msg_length += ((int)buf[i])<<8; read_state++;
          msg_ptr = 0;
          if (msg_length > sizeof(msgbuf)) {
            fprintf(stderr, "discarding (%02x,%02x) message of length %d "
                    "(buffer is %d)\n",
                    msg_cls, msg_id, msg_length, sizeof(msgbuf));
            read_state = 0;
          }
          if (msg_length == 0) read_state++;  // skip payload read
          break;
        case 6:
          msgbuf[msg_ptr++] = buf[i];
          if (msg_ptr == msg_length) read_state++;
          break;
        case 7:
          if (msg_cka != buf[i]) {
            fprintf(stderr, "discarding (%02x,%02x) message; "
                    "cka mismatch (got %02x calc'd %02x)\n",
                    msg_cls, msg_id, buf[i], msg_cka);
            read_state = 0;
          } else
            read_state++;
          break;
        case 8:
          if (msg_ckb != buf[i]) {
            fprintf(stderr, "discarding (%02x,%02x) message; "
                    "cka mismatch (got %02x calc'd %02x)\n",
                    msg_cls, msg_id, buf[i], msg_cka);
          } else {
            process_msg(fd, msg_cls, msg_id, msgbuf, msg_length);
          }
          read_state = 0;
          break;
      }
    }
    // printf("\n");
  }
}

void init_ubx_protocol(int fd) {
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

  // now send baudrate-changing message
  char nmeamsg[256];
  snprintf(nmeamsg, sizeof(nmeamsg),
           "$PUBX,41,1,0007,0001,%d,0", runtime_baudrate);
  nmea_sendmsg(fd, nmeamsg);

  // add some guard time before and after changing baud rates
  usleep(100000);
  cfsetspeed(&tios, runtime_ioctl_baud);
  tcsetattr(fd, TCSADRAIN, &tios);
}

int main(int argc, char** argv) {
  // TODO: open serial port, set baudrate 9600,
  // send NMEA-ish UBX message to switch to 115200 + binary protocol
  // configure 5Hz GPS events
  // display coords, velocity
  int fd = open(ubx_port, O_RDWR);  // O_NONBLOCK?
  if (fd == -1) {
    perror(ubx_port);
    return 1;
  }
  if (argc < 2) {  // specify an extra argument to warmstart/skip setup
    init_ubx_protocol(fd);
    close(fd);
    fd = open(ubx_port, O_RDWR);
  }

  ubx_sendmsg(fd, 6, 6, NULL, 0);

  // ubx_enable_periodic(fd, 1,2, 1);  // enable NAV-POSLLH
  ubx_enable_periodic(fd, 1,1, 1);  // enable NAV-POSECEV

  read_loop(fd);

  return 0;
}
