#include <stdio.h>

#include "./sirf.h"

int main(int argc, char** argv) {
  int fd = sirf_open();
  if (fd == -1) return 1;

  sirf_read_loop(fd);

  return 0;
}
