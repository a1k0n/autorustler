#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "cam/cam.h"
#include "ui/uistate.h"

class UICamReceiver: public CameraReceiver {
 public:
  void OnFrame(uint8_t *buf, size_t length) {
    struct timeval t;
    gettimeofday(&t, NULL);

    // TODO: if(recording) ... add message

    int idxout = 64*48;
    for (int y = 0; y < 240; y += 5) {
      int idxin = y*320;
      for (int x = 0; x < 320; x += 5) {
        uistate.cam_preview[--idxout] = buf[idxin];
        idxin += 5;
      }
    }
  }
};

static UICamReceiver r;

bool StartCamera() {
  if (!Camera::Init(320, 240, 20)) {
    fprintf(stderr, "camera init fail\n");
    return 1;
  }

  Camera::StartRecord(&r);
}
