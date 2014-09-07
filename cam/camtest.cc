#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include "cam/cam.h"

class Recorder: public CameraReceiver {
 public:
  Recorder() { output_file_ = NULL; frame_ = 0; }

  bool Init(const char *fname) {
    output_file_ = fopen(fname, "wb");
    if (!output_file_) {
      perror(fname);
      return false;
    }
    return true;
  }

  ~Recorder() {
    if (output_file_) fclose(output_file_);
  }

  void OnFrame(uint8_t *buf, size_t length) {
    struct timeval t;
    gettimeofday(&t, NULL);
    fwrite(buf, 1, length, output_file_);
    fprintf(stderr, "%d.%06d frame %d\n", t.tv_sec, t.tv_usec, frame_++);
  }

 private:
  FILE *output_file_;
  int frame_;
};

int main() {
  if (!Camera::Init(320, 240, 10))
    return 1;
  struct timeval t;
  gettimeofday(&t, NULL);
  fprintf(stderr, "%d.%06d started camera\n", t.tv_sec, t.tv_usec);

  Recorder r;
  if (!r.Init("output.yuv"))
    return 1;

  sleep(1);  // wait one second for auto-exposure to come up to speed

  if (!Camera::StartRecord(&r))
    return 1;

  sleep(1);

  Camera::StopRecord();
}
