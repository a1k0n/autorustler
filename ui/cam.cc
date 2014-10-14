#include <stdio.h>
#include <stdlib.h>

#include "cam/cam.h"
#include "ui/uistate.h"

class UICamReceiver: public CameraReceiver {
 public:
  void OnFrame(uint8_t *buf, size_t length) {
    if (uistate.is_recording) {
      RecordHeader rh;
      rh.Init(length, 1);
      recording.StartWriting();
      recording.Write(reinterpret_cast<uint8_t*>(&rh), sizeof(rh));
      recording.Write(buf, length);
      recording.StopWriting();
    }

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
