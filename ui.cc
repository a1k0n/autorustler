#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>

#include "imu/imu.h"
#include "lcd/gpio.h"
#include "lcd/lcd.h"
#include "./radio.h"

volatile bool done = false;

void handle_sigint(int signo) { done = true; }

int main() {
  if (!gpio_init()) {
    fprintf(stderr, "gpio init fail\n");
    return 1;
  }

  LCD lcd;
  if (!lcd.Init()) {
    fprintf(stderr, "lcd init fail\n");
    return 1;
  }

  RadioControl rc;
  if (!rc.Init()) {
    fprintf(stderr, "radio init fail\n");
    return 1;
  }

  int i2cfd = open("/dev/i2c-1", O_RDWR);
  if (i2cfd == -1) {
    perror("/dev/i2c-1");
    return 1;
  }

  signal(SIGINT, handle_sigint);

  if (imu_init(i2cfd)) {
    fprintf(stderr, "imu init fail\n");
    return 1;
  }

  uint8_t screen[84*6];

  imu_state minstate, maxstate;
  bool stateinit = false;

  while (!done) {
    memset(screen, 0, sizeof(screen));
    imu_state s;
    RCState rcstate;
    imu_read(i2cfd, &s);
    rc.GetRadioState(&rcstate);
    if (!stateinit) {
      minstate = s;
      maxstate = s;
      stateinit = true;
      continue;
    } else {
      minstate.gyro_x = std::min(minstate.gyro_x, s.gyro_x);
      maxstate.gyro_x = std::max(maxstate.gyro_x, s.gyro_x);
      minstate.gyro_y = std::min(minstate.gyro_y, s.gyro_y);
      maxstate.gyro_y = std::max(maxstate.gyro_y, s.gyro_y);
      minstate.gyro_z = std::min(minstate.gyro_z, s.gyro_z);
      maxstate.gyro_z = std::max(maxstate.gyro_z, s.gyro_z);
      minstate.mag_x = std::min(minstate.mag_x, s.mag_x);
      maxstate.mag_x = std::max(maxstate.mag_x, s.mag_x);
      minstate.mag_y = std::min(minstate.mag_y, s.mag_y);
      maxstate.mag_y = std::max(maxstate.mag_y, s.mag_y);
      minstate.mag_z = std::min(minstate.mag_z, s.mag_z);
      maxstate.mag_z = std::max(maxstate.mag_z, s.mag_z);
    }

    // draw a compass, just a dot over an 'N', 28x24
    LCD::WriteString(11, 8, "N", screen);
    int x = 28*(s.mag_x - minstate.mag_x) /
        (maxstate.mag_x - minstate.mag_x + 1);
    int y = 24 - 24*(s.mag_z - minstate.mag_z) /
        (maxstate.mag_z - minstate.mag_z + 1);
    screen[(y/8)*84 + x] |= 1 << (y & 7);

    char buf[16];
    snprintf(buf, sizeof(buf), "gx %d", s.gyro_x);
    LCD::WriteString(28, 0, buf, screen);
    snprintf(buf, sizeof(buf), "gy %d", s.gyro_y);
    LCD::WriteString(28, 8, buf, screen);
    snprintf(buf, sizeof(buf), "gz %d", s.gyro_z);
    LCD::WriteString(28, 16, buf, screen);

    float vbat = 0;
    rc.GetBatteryVoltage(&vbat);
    snprintf(buf, sizeof(buf), "%0.1fV", vbat);
    LCD::WriteString(59, 40, buf, screen);

    screen[3*84 + rcstate.throttle/4] = 0xff;
    screen[4*84 + rcstate.steering/4] = 0xff;

    lcd.GotoXY(0, 0);
    lcd.Draw(screen, 84*6);
    usleep(20000);
  }

  memset(screen, 0, sizeof(screen));
  lcd.GotoXY(0, 0);
  lcd.Draw(screen, 84*6);

  return 0;
}
