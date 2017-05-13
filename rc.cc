#include <stdio.h>
#include <unistd.h>

#include "car/pca9685.h"
#include "gpio/i2c.h"
#include "input/js.h"

int main() {
  I2C i2c;
  JoystickInput js;

  if (!i2c.Open()) {
    return 1;
  }

  if (!js.Open()) {
    return 1;
  }

  PCA9685 pca(i2c);

  pca.Init(100);  // 100Hz output
  pca.SetPWM(0, 614.4);
  pca.SetPWM(1, 614.4);

  for (;;) {
    int t = 0, s = 0;
    uint16_t b = 0;
    uint16_t last_esc = 0;
    if (js.ReadInput(&t, &s, &b)) {
      pca.SetPWM(0, 614.4 - 204.8*s / 32767.0);  // steering is backwards
      uint16_t esc = 614.4 + 204.8*t / 32767.0;
      pca.SetPWM(1, esc);
      if (esc != last_esc) {
        last_esc = esc;
        printf("%d\n", esc);
      }
    }
    usleep(1000);
  }
}
