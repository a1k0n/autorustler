#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "cam/cam.h"
#include "car/pca9685.h"
#include "gpio/i2c.h"
#include "input/js.h"

volatile bool done = false;
uint16_t throttle_ = 0, steering_ = 0;

void handle_sigint(int signo) { done = true; }

I2C i2c;
PCA9685 pca(i2c);

class Driver: public CameraReceiver {
 public:
  Driver() {
    output_file_ = NULL;
    frame_ = 0;
    autosteer_ = false;
  }

  bool StartRecording(const char *fname) {
    if (!strcmp(fname, "-")) {
      output_file_ = stdout;
      setbuf(stdout, NULL);
    } else {
      output_file_ = fopen(fname, "wb");
    }
    if (!output_file_) {
      perror(fname);
      return false;
    }
    return true;
  }

  bool IsRecording() {
    return output_file_ != NULL;
  }

  void StopRecording() {
    FILE *f = output_file_;
    output_file_ = NULL;
    if (f == NULL) {
      return;
    }
    if (f != stdout) {
      fclose(f);
    }
    output_file_ = NULL;
  }

  ~Driver() {
    StopRecording();
  }

  void OnFrame(uint8_t *buf, size_t length) {
    if (output_file_) {
      struct timeval t;
      gettimeofday(&t, NULL);
      fwrite(&t.tv_sec, sizeof(t.tv_sec), 1, output_file_);
      fwrite(&t.tv_usec, sizeof(t.tv_usec), 1, output_file_);
      fwrite(&throttle_, sizeof(throttle_), 1, output_file_);
      fwrite(&steering_, sizeof(steering_), 1, output_file_);
      fwrite(buf, 1, length, output_file_);
    }

    // identify yellow lines, fit line, PID steering!
    // simple AF
    // we don't even need to undistort for that, though we could

    const int ytop = 274 / 2;
    // linear regression statistics:
    float sumxy = 0, sumx = 0, sumy = 0, sumy2 = 0, n = 0;
    for (int y = ytop; y < 240; y++) {
      for (int x = 0; x < 320; x++) {
        uint8_t u = buf[640*480 + y*320 + x];
        if (u >= 105) continue;
        uint8_t v = buf[640*480 + 320*240 + y*320 + x];
        if (v <= 140) continue;
#if 0
        uint8_t y = buf[y*640 + x*2];
        if (y < 160) continue;
#endif
        // add x, y to linear regression
        sumx += x;
        sumy += (240.-y);
        sumxy += (240.-y)*x;
        sumy2 += (240.-y)*(240.-y);
        n += 1;
      }
    }
#if 0
    sumx *= 0.5;
    sumy *= 0.5;
    sumxy *= 0.5;
    sumy2 *= 0.5;
    n *= 0.5;
#endif
    static int last_side = 0;

    static const float CONST_P = 1.0 / 500.0;
    static const float CONST_V = 700;  // safe speed: 700

    if (n > 5) {
      // intercept is the projection of the line on the bottom of the screen
      // which is our steering error
      // the slope also determines our angle offset
      sumx /= n;
      sumy /= n;
      sumxy /= n;
      sumy2 /= n;
      float beta = (sumxy - sumx*sumy) / (sumy2 - sumy*sumy);
      float alpha = sumx - beta*sumy;
      fprintf(stderr, "alpha %f beta %f\n", alpha, beta);
      last_side = alpha > 160 ? 1 : -1;
      if (autosteer_) {
        float s = CONST_P * (alpha - 160);
        float s0 = -6500 / 32767.0;
        steering_ = 614.4 - 204.8*(s0 + s);
        pca.SetPWM(0, steering_);
        pca.SetPWM(1, CONST_V);
      }
    } else if (autosteer_) {
      // we're lost, so we have to assume we're way off the same side of the
      // screen as last time
      pca.SetPWM(0, 614.4 - 204.8 * last_side);
      pca.SetPWM(1, CONST_V - 20);
    }
  }

  bool autosteer_;

 private:
  FILE *output_file_;
  int frame_;
};

int main(int argc, char *argv[]) {
  signal(SIGINT, handle_sigint);

  if (argc < 2) {
    fprintf(stderr, "%s [[output.yuv] [fps]\n", argv[0]);
    return 1;
  }

  int fps = 30;

  if (argc > 2) {
    fps = atoi(argv[2]);
    if (fps == 0) {
      fprintf(stderr, "invalid fps %d\n", fps);
      return 1;
    }
  }

  if (!Camera::Init(640, 480, fps))
    return 1;

  JoystickInput js;

  if (!i2c.Open()) {
    return 1;
  }

  if (!js.Open()) {
    return 1;
  }


  pca.Init(100);  // 100Hz output
  pca.SetPWM(0, 614);
  pca.SetPWM(1, 614);

  struct timeval tv;
  gettimeofday(&tv, NULL);
  fprintf(stderr, "%d.%06d camera on @%d fps\n", tv.tv_sec, tv.tv_usec, fps);

  Driver driver;
  if (!Camera::StartRecord(&driver))
    return 1;

  gettimeofday(&tv, NULL);
  fprintf(stderr, "%d.%06d started camera\n", tv.tv_sec, tv.tv_usec);


  while (!done) {
    int t = 0, s = 0;
    uint16_t b = 0;
    if (js.ReadInput(&t, &s, &b)) {
      gettimeofday(&tv, NULL);
      if (b & 0x40) {  // start button: start recording
        if (driver.StartRecording(argv[1])) {
          fprintf(stderr, "%d.%06d started recording %s\n", tv.tv_sec, tv.tv_usec, argv[1]);
        }
      }
      if ((b & 0x30) == 0x30) {
        driver.StopRecording();
        fprintf(stderr, "%d.%06d stopped recording\n", tv.tv_sec, tv.tv_usec);
      }

      if (b & 0x10) {  // not sure which button this is
        if (!driver.autosteer_) {
          fprintf(stderr, "%d.%06d autosteer ON\n", tv.tv_sec, tv.tv_usec);
          driver.autosteer_ = true;
        }
      } else {
        if (driver.autosteer_) {
          driver.autosteer_ = false;
          fprintf(stderr, "%d.%06d autosteer OFF\n", tv.tv_sec, tv.tv_usec);
        }
      }

      if (!driver.autosteer_) {
        steering_ = 614.4 - 204.8*s / 32767.0;
        pca.SetPWM(0, steering_);
        throttle_ = 614.4 + 204.8*t / 32767.0;
        pca.SetPWM(1, throttle_);
      }
    }
    usleep(1000);
  }

  Camera::StopRecord();
}
