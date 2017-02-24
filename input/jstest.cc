#include <stdio.h>
#include <unistd.h>

#include "input/js.h"

int main() {
  JoystickInput js;

  if (!js.Open()) {
    return 1;
  }

  for (;;) {
    int t = 0, s = 0;
    if (js.ReadInput(&t, &s)) {
      printf("%6d %6d\r", t, s);
      fflush(stdout);
    }
    usleep(10000);
  }
}
