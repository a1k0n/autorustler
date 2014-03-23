#include <signal.h>
#include <stdio.h>
#include <sys/time.h>

#include "./sirf.h"
#include "./vec3.h"

vec3<int32_t> refpos;
bool have_refpos = false;
volatile bool done = false;

void handle_sigint(int signo) { done = true; }

void update(const sirf_navdata& data) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  if (!have_refpos) {
    refpos = vec3<int32_t>(data.x, data.y, data.z);
    have_refpos = true;
    printf("# refpos %d %d %d\n", data.x, data.y, data.z);
  } else {
    vec3<int32_t> pos(data.x, data.y, data.z);
    pos -= refpos;
    printf("%d.%06d %d %d %d %0.2f %0.2f %0.2f %0.1f %d\n",
           tv.tv_sec, tv.tv_usec,
           pos.x, pos.y, pos.z,
           data.v8x/8.0f, data.v8y/8.0f, data.v8z/8.0f,
           data.hdop/5.0f, data.svs);
  }
  fprintf(stderr, "[%d, %d, %d] velocity [%0.2f, %0.2f, %0.2f] "
          "hdop=%0.1f, SVs=%d\e[K\r",
         data.x, data.y, data.z,
         data.v8x/8.0f, data.v8y/8.0f, data.v8z/8.0f,
         data.hdop/5.0f, data.svs);
  fflush(stderr);
}

int main(int argc, char** argv) {
  signal(SIGINT, handle_sigint);

  int fd = sirf_open();
  if (fd == -1) return 1;

  while (!done && sirf_poll(fd, update)) {}

  return 0;
}
