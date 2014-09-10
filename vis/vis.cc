#include <stdio.h>
#include <algorithm>
#include <vector>

#include "SDL/SDL.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"


using std::min;
using std::max;
using std::vector;

static inline int clamp(int x) {
  return max(0, min(255, x));
}

bool Poll() {
  SDL_Event event;

  while (SDL_PollEvent(&event)) {
    switch (event.type) {
      case SDL_KEYDOWN:
        return false;
    }
  }
  return true;
}

void RenderFrame(const uint8_t *yuvbuf, SDL_Surface *frame) {
  uint32_t *pixbuf = reinterpret_cast<uint32_t*>(frame->pixels);
  for (int j = 0; j < 240; j++) {
    // the camera is rotated 180 degrees, so render it upside-down
    // (bottom-to-top, right-to-left)
    int idx = (240 - j) * 320;
    for (int i = 0; i < 320; i++) {
      int c = yuvbuf[j*320 + i] - 16;
      int d = yuvbuf[320*240 + (j>>1)*160 + (i>>1)] - 128;
      int e = yuvbuf[320*300 + (j>>1)*160 + (i>>1)] - 128;
      pixbuf[--idx] = 0xff000000 |
          (clamp((298*c + 409*e + 128) >> 8)) |
          (clamp((298*c - 100*d - 208*e + 128) >> 8) << 8) |
          (clamp((298*c + 516*d + 128) >> 8) << 16);
    }
  }

  // construct an opencv Mat
  cv::Mat uimage(240, 320, CV_8UC1, const_cast<uint8_t*>(yuvbuf));

  vector<cv::KeyPoint> points(100);
  cv::FastFeatureDetector fastdet(5);

  // put a green pixel on each keypoint
  fastdet.detect(uimage, points);
  for (int i = 0; i < points.size(); i++) {
    int x = points[i].pt.x;
    int y = points[i].pt.y;
    pixbuf[(240 - y) * 320 - x - 1] = 0xff00ff00;
    pixbuf[(240 - y) * 320 - x - 2] = 0xff00ff00;
    pixbuf[(240 - y) * 320 - x] = 0xff00ff00;
    pixbuf[(239 - y) * 320 - x - 1] = 0xff00ff00;
    pixbuf[(241 - y) * 320 - x - 1] = 0xff00ff00;
  }
}

int SDL_main(int argc, char *argv[]) {
  if (argc < 2) {
    fprintf(stderr, "%s [output.yuv]\n", argv[0]);
    return 1;
  }

  FILE *fp = fopen(argv[1], "rb");
  if (!fp) {
    perror(argv[1]);
    return 1;
  }

  SDL_Init(SDL_INIT_VIDEO);

  SDL_Surface *screen = SDL_SetVideoMode(320, 240, 32, SDL_SWSURFACE);
  if (!screen) {
    fprintf(stderr, "sdl screen init fail");
    return 1;
  }
  SDL_WM_SetCaption("autorustler viz", NULL);

  SDL_Surface *frame = SDL_CreateRGBSurface(
      SDL_SWSURFACE, 320, 240, 32,
      0x000000ff, 0x0000ff00, 0x00ff0000, 0xff000000);

  SDL_Rect rect320240 = {0, 0, 320, 240};

  int frameno = 0;
  for (;;) {
    uint32_t sec, usec;
    uint8_t yuvbuf[320*360];
    if (fread(&sec, 1, 4, fp) < 4)
      break;
    if (fread(&usec, 1, 4, fp) < 4)
      break;
    if (fread(yuvbuf, 1, sizeof(yuvbuf), fp) < sizeof(yuvbuf))
      break;
    SDL_LockSurface(frame);
    RenderFrame(yuvbuf, frame);
    SDL_UnlockSurface(frame);
    SDL_BlitSurface(frame, NULL, screen, NULL);
    SDL_Flip(screen);
    if (!Poll())
      break;
    SDL_Delay(50);
  }

  return 0;
}
