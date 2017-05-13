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

    c.PredictStep(u_a, u_s, 1.0/30.0);
    std::cout << "predict x" << c.x_.transpose() << std::endl;
    std::cout << "predict P" << c.P_.diagonal().transpose() << std::endl;
    c.UpdateCamera(framebuf+36);
    std::cout << "camera x" << c.x_.transpose() << std::endl;
    std::cout << "camera P" << c.P_.diagonal().transpose() << std::endl;
    c.UpdateIMU(a, g, u_a);
    std::cout << "IMU x" << c.x_.transpose() << std::endl;
    std::cout << "IMU P" << c.P_.diagonal().transpose() << std::endl;

    // just enforce boundaries
    c.UpdateState(framebuf+36, sizeof(framebuf), u_a, u_s, a, g);
  }
}

