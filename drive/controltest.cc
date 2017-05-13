#include <iostream>
#include <stdio.h>
#include "drive/controller.h"

int main() {
  DriveController c;

  uint8_t framebuf[640*480 + 320*240*2 + 36];

  while (fread(framebuf, 1, sizeof(framebuf), stdin) == sizeof(framebuf)) {
    uint16_t controls[2];
    float accel[3];
    float gyro[3];
    memcpy(controls, framebuf+8, 4);
    memcpy(accel, framebuf+12, 12);
    memcpy(gyro, framebuf+24, 12);
    Eigen::Vector3f a(accel), g(gyro);

    float u_a = controls[0] / 204.8 - 3.0;
    float u_s = controls[1] / 204.8 - 3.0;
    std::cout << "controls: " << u_a << " " << u_s
      << " accel " << a.transpose() << " gyro " << g.transpose() << std::endl;

    c.UpdateState(framebuf+36, sizeof(framebuf) - 36, u_a, u_s, a, g);

    std::cout << "x " << c.x_.transpose() << std::endl;
    std::cout << "P " << c.P_.diagonal().transpose() << std::endl;

    c.GetControl(&u_a, &u_s);
    std::cout << "control " << u_a << " " << u_s << std::endl;
  }
}

