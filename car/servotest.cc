#include <math.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "car/pca9685.h"
#include "gpio/i2c.h"

int main() {
  I2C i2c;

  if (!i2c.Open()) {
    return 1;
  }

  PCA9685 pca(i2c);

  pca.Init(100);  // 100Hz output

  // pwm period is 10ms
  // servo range is 1..2ms, so 409.6 .. 819.2
  // center both output channels
  pca.SetPWM(0, 614.4);
  pca.SetPWM(1, 614.4);

  int i = 0;
  for (;;) {
    pca.SetPWM(0, 614.4 + 200*sin(i * 0.01));
    i += 1;
    usleep(10000);
  }
}
