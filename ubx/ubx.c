#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

const char ubx_port[] = "/dev/ttyAMA0";
// #define startup_baudrate 9600
#define startup_ioctl_baud B115200
#define runtime_baudrate 115200
#define runtime_ioctl_baud B115200

const char rate_msg[] = "PUBX,41,1,0007,0003,%d,0";  // i don't think we need this?

void init_ubx_protocol(int fd) {
  struct termios tios;
  tcgetattr(fd, &tios);
  cfsetspeed(&tios, startup_ioctl_baud);
  tcsetattr(fd, TCSADRAIN, &tios);

  // now send baudrate-changing message

  tcgetattr(fd, &tios);
  cfsetspeed(&tios, runtime_ioctl_baud);
  tcsetattr(fd, TCSADRAIN, &tios);
}

// writes CK_A and CK_B into buf[len] and buf[len+1]
void ubx_checksum(uint8_t* buf, int len) {
  uint8_t cka = 0, ckb = 0;
  int i;
  for (i = 2; i < len-2; i++) {
    cka += buf[i];
    ckb += cka;
  }
  buf[len-2] = cka;
  buf[len-1] = ckb;
}

void process_msg(int msg_class, int msg_id, uint8_t *msgbuf, int msg_length) {
  int i;
  printf("read msg (%02x,%02x): ", msg_class, msg_id);
  for (i = 0; i < msg_length; i++)
    printf("%02x ", msgbuf[i]);
  printf("\n");
}

void read_loop(int fd) {
  uint8_t buf[256];
  static uint8_t msgbuf[512];
  static int read_state = 0;
  static int msg_length = 0, msg_ptr = 0;
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
      if (read_state > 2 && read_state < 7) {
        msg_cka += buf[i];
        msg_ckb += msg_cka;
      }
      switch (read_state) {
        case 0:
          if (buf[i] == 0xb5) read_state++;
          break;
        case 1:
          if (buf[i] == 0x62) read_state++;
          else if (buf[i] != 0xb5) read_state = 0;
          break;
        case 2:
          msg_cls = buf[i]; read_state++;
          msg_cka = buf[i]; msg_ckb = msg_cka;
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
            process_msg(msg_cls, msg_id, msgbuf, msg_length);
          }
          read_state = 0;
          break;
      }
    }
  }
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
  }

  uint8_t ubx_buf[256] = {
      0xb5, 0x62,  // sync
      1, 6,        // NAV-SOL
//      6, 0,        // CFG-PRT
//      20, 0,       // length (?)
      0, 0,       // length (?)
      1, 0,        // port 1 (UART), reserved
      0x00, 0x00,  // txReady = 0
      0x8d, 0,0,0, // mode = 8 data bits, 1 stop bit, no parity
      0, 0, 0, 0,  // baudrate [filled in below]
      1, 0, 1, 0,  // in/outProtoMask = UBX only
      0, 0, 0, 0   // reserved
  };
  ubx_buf[6+8] = runtime_baudrate & 0xff;
  ubx_buf[6+9] = (runtime_baudrate >> 8) & 0xff;
  ubx_buf[6+10] = (runtime_baudrate >> 16) & 0xff;
  ubx_buf[6+11] = (runtime_baudrate >> 24) & 0xff;

  int len = 8;
  ubx_checksum(ubx_buf, len);

  //int len = 28
  //ubx_checksum(ubx_buf, len);
  int i;
  for(i = 0; i < len; i++) {
    printf("%02x ", ubx_buf[i]);
  }
  printf("\n");
#if 1
  len = write(fd, ubx_buf, len);
  if (len == -1) {
    perror("write");
  }

  read_loop(fd);
#endif

  return 0;
}
