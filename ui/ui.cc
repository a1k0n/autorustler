#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>

#include "car/radio.h"
#include "gpio/gpio.h"
#include "gps/sirf.h"
#include "imu/imu.h"
#include "lcd/lcd.h"
#include "ui/uistate.h"

volatile UIState uistate;

void handle_sigint(int signo) { uistate.done = true; }

const int BUTTON_RED = 24;     // GPIO24 -> red button
const int BUTTON_BLACK = 23;   // GPIO23 -> black button

extern void* GPSThread(void* data);
extern void* IMUThread(void* data);
extern bool StartCamera();

void InitButtons() {
  INP_GPIO(BUTTON_RED);
  INP_GPIO(BUTTON_BLACK);
  GPIO_PULL = 2;
  usleep(10);
  // clock on GPIO 23 & 24 (bit 23 & 24 set)
  GPIO_PULLCLK0 = (1 << BUTTON_RED) | (1 << BUTTON_BLACK);
  usleep(10);
  GPIO_PULL = 0;
  GPIO_PULLCLK0 = 0;
}

const uint8_t dither_matrix[4][4] = {
  { 1*15,  9*15,  3*15, 11*15},
  {13*15,  5*15, 15*15,  7*15},
  { 4*15, 12*15,  2*15, 10*15},
  {16*15,  8*15, 14*15,  6*15}
};

int main() {
  if (!gpio_init()) {
    fprintf(stderr, "gpio init fail\n");
    return 1;
  }

  InitButtons();

  LCD lcd;
  if (!lcd.Init()) {
    fprintf(stderr, "lcd init fail\n");
    return 1;
  }

  int i2c_fd = open("/dev/i2c-1", O_RDWR);
  if (i2c_fd == -1) {
    perror("/dev/i2c-1");
    return 1;
  }

  // TODO: launch IMU/radio thread

  int gps_fd = sirf_open();
  if (gps_fd == -1) {
    fprintf(stderr, "GPS serial port init fail\n");
    return 1;
  }
  pthread_t gps_thread;
  pthread_create(&gps_thread, NULL, GPSThread,
                 reinterpret_cast<void*>(gps_fd));

  pthread_t imu_thread;
  pthread_create(&imu_thread, NULL, IMUThread,
                 reinterpret_cast<void*>(i2c_fd));

  if (!StartCamera()) {
    fprintf(stderr, "cam init fail\n");
    return 1;
  }

  signal(SIGINT, handle_sigint);
  signal(SIGTERM, handle_sigint);

  uint8_t screen[84*6];

  char buf[84];
  while (!uistate.done) {
    memset(screen, 0, sizeof(screen));

    // draw camera frame
    for (int y = 0, idxin = 0; y < 48; y += 8) {
      int idxout = (y>>3)*84;
      for (int x = 0; x < 64; x++) {
        uint8_t pxl = 0;
        for (int i = 0; i < 8; i++) {
          if (uistate.cam_preview[idxin++] < dither_matrix[i&3][x&3]) {
            pxl |= (1 << i);
          }
        }
        screen[idxout++] = pxl;
      }
    }

    snprintf(buf, sizeof(buf), "%0.1fV", uistate.vbat);
    LCD::WriteString(59, 40, buf, screen);

    screen[3*84 + uistate.rc_state.throttle/4] = 0xff;
    screen[4*84 + uistate.rc_state.steering/4] = 0xff;

    if (GET_GPIO(BUTTON_BLACK) == 0) {
      screen[0] = 0x1c;
      screen[1] = 0x3e;
      screen[2] = 0x3e;
      screen[3] = 0x1c;
    }
    if (GET_GPIO(BUTTON_RED) == 0) {
      screen[5] = 0x1c;
      screen[6] = 0x3e;
      screen[7] = 0x3e;
      screen[8] = 0x1c;
    }

    lcd.GotoXY(0, 0);
    lcd.Draw(screen, 84*6);
    usleep(100000);
  }

  memset(screen, 0, sizeof(screen));
  lcd.GotoXY(0, 0);
  lcd.Draw(screen, 84*6);

  return 0;
}
