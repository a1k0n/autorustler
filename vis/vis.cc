#include <stdio.h>
#include <SDL.h>
#include <algorithm>

using std::min;
using std::max;

static inline int clamp(int x) {
  return max(0, min(255, x));
}

bool poll() {
  SDL_Event event;

  while (SDL_PollEvent(&event)) {
    switch (event.type) {
      case SDL_KEYDOWN:
        return false;
    }
  }
  return true;
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
    printf("frame %d %ld\n", ++frameno, ftell(fp));
    SDL_LockSurface(frame);
    uint32_t *pixbuf = reinterpret_cast<uint32_t*>(frame->pixels);
    for (int j = 0, idx = 0; j < 240; j++) {
      for (int i = 0; i < 320; i++) {
        int c = yuvbuf[j*320 + i] - 16;
        int d = yuvbuf[320*240 + (j>>1)*160 + (i>>1)] - 128;
        int e = yuvbuf[320*300 + (j>>1)*160 + (i>>1)] - 128;
        pixbuf[idx++] = 0xff000000 |
            (clamp((298*c + 409*e + 128) >> 8)) |
            (clamp((298*c - 100*d - 208*e + 128) >> 8) << 8) |
            (clamp((298*c + 516*d + 128) >> 8) << 16);
      }
    }
    SDL_UnlockSurface(frame);
    SDL_BlitSurface(frame, NULL, screen, NULL);
    SDL_Flip(screen);
    if (!poll())
      break;
    SDL_Delay(50);
  }

  return 0;
}
