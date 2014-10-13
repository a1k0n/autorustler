#include <stdio.h>
#include <sys/time.h>

#include "gps/sirf.h"
#include "gps/vec3.h"
#include "ui/uistate.h"

vec3<int32_t> refpos;
bool have_refpos = false;

void GPSUpdate(const sirf_navdata& data) {
  // TODO: if (recording) ... log gps state
  if (!have_refpos) {
    refpos = vec3<int32_t>(data.x, data.y, data.z);
    have_refpos = true;
    printf("# refpos %d %d %d\n", data.x, data.y, data.z);
  } else {
    vec3<int32_t> pos(data.x, data.y, data.z);
    pos -= refpos;
    uistate.gps_x = pos.x;
    uistate.gps_y = pos.y;
    uistate.gps_z = pos.z;
#if 0
    struct timeval tv;
    gettimeofday(&tv, NULL);
    printf("%d.%06d %d %d %d %d %d %0.2f %0.2f %0.2f %0.1f %d\n",
           tv.tv_sec, tv.tv_usec,
           data.gps_week, data.gps_tow,
           pos.x, pos.y, pos.z,
           data.v8x/8.0f, data.v8y/8.0f, data.v8z/8.0f,
           data.hdop/5.0f, data.svs);
#endif
  }
#if 0
  int tow = data.gps_tow - 1600;  // adjust for leap seconds to get UTC
  int jif = tow % 100; tow /= 100;
  int sec = tow % 60; tow /= 60;
  int min = tow % 60; tow /= 60;
  int hour = tow % 24;
  fprintf(stderr, "[%02d:%02d:%02d.%02d] pos=[%d, %d, %d] "
          "vel=[%0.2f, %0.2f, %0.2f] hdop=%0.1f, SVs=%d\n",
         hour, min, sec, jif,
         data.x, data.y, data.z,
         data.v8x/8.0f, data.v8y/8.0f, data.v8z/8.0f,
         data.hdop/5.0f, data.svs);
#endif
  uistate.gps_SVs = data.svs;

  RecordHeader rh;
  rh.Init(sizeof(data), 3);
  recording.StartWriting();
  recording.Write(reinterpret_cast<const uint8_t*>(&rh), sizeof(rh));
  recording.Write(reinterpret_cast<const uint8_t*>(&data), sizeof(data));
  recording.StopWriting();
}

void* GPSThread(void* data) {
  int fd = reinterpret_cast<int>(data);
  while (!uistate.done && sirf_poll(fd, GPSUpdate)) {}
  return NULL;
}
