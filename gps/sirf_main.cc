#include <stdio.h>

#include "./sirf.h"
#include "./vec3.h"

vec3<int32_t> refpos;
bool have_refpos = false;

void update(const sirf_navdata& data) {
  if (!have_refpos) {
    refpos = vec3<int32_t>(data.x, data.y, data.z);
    have_refpos = true;
  } else {
    vec3<int32_t> pos(data.x, data.y, data.z);
    pos -= refpos;
    printf("%d %d %d\n", pos.x, pos.y, pos.z);
    fflush(stdout);
  }
  fprintf(stderr, "[%d, %d, %d] velocity [%0.2f, %0.2f, %0.2f] "
          "hdop=%0.1f, SVs=%d\n",
         data.x, data.y, data.z,
         data.v8x/8.0f, data.v8y/8.0f, data.v8z/8.0f,
         data.hdop/5.0f, data.svs);
}

int main(int argc, char** argv) {
  int fd = sirf_open();
  if (fd == -1) return 1;

  while (sirf_poll(fd, update)) {}

  return 0;
}
