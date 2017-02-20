#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "gpio/i2c.h"
#include "imu/imu.h"

volatile bool done = false;

void handle_sigint(int signo) { done = true; }

int main() {
  I2C i2c;
  if (!i2c.Open()) {
    return 1;
  }

  signal(SIGINT, handle_sigint);

  IMU imu(i2c);
  printf("# t gx gy gz mx my mz ax ay az\n");

  bool mag_calibrated = imu.LoadMagCalibration();

  while (!done) {
    timeval tv0;
    gettimeofday(&tv0, NULL);
    IMUState s;
    imu.ReadCalibrated(&s);

    printf("%d.%06d m [%f,%f,%f] a [%f,%f,%f] g [%f,%f,%f]\n",
        tv0.tv_sec, tv0.tv_usec,
        s.N[0], s.N[1], s.N[2],
        s.g[0], s.g[1], s.g[2],
        s.w[0], s.w[1], s.w[2]);

    fflush(stdout);
    timeval tv;
    gettimeofday(&tv, NULL);
    int delay = 20000 - (tv.tv_usec % 20000);
    if (delay > 0)
      usleep(delay);
  }

  return 0;
}
