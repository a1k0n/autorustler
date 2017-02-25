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

class Driver: public CameraReceiver {
 public:
  Driver() { output_file_ = NULL; frame_ = 0; }

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
    // fprintf(stderr, "%d.%06d frame %d\n", t.tv_sec, t.tv_usec, frame_++);
  }

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

  int fps = 20;

  if (argc > 2) {
    fps = atoi(argv[2]);
    if (fps == 0) {
      fprintf(stderr, "invalid fps %d\n", fps);
      return 1;
    }
  }

  if (!Camera::Init(640, 512, fps))
    return 1;

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
      steering_ = 614.4 - 204.8*s / 32767.0;
      throttle_ = 614.4 + 204.8*t / 32767.0;
      pca.SetPWM(0, steering_);
      pca.SetPWM(1, throttle_);
    }
    usleep(1000);
  }

  Camera::StopRecord();
}
