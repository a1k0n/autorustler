#include <stdio.h>
#include <algorithm>
#include <vector>

#include "SDL/SDL.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "fast/fast.h"
#include "vikit/vision.h"

using std::min;
using std::max;
using std::vector;

const int n_pyr_levels = 4;

class ImagePyramid {
 public:
  ImagePyramid() { pyr_ = NULL; }
  ~ImagePyramid() { delete[] pyr_; }
  void Init(const uint8_t *framebuf, int n_levels) {
    pyr_ = new cv::Mat[n_levels];
    pyr_[0] = cv::Mat(240, 320, CV_8UC1);
    memcpy(pyr_[0].data, framebuf, 320*240);
    for (int i = 1; i < n_levels; i++) {
      pyr_[i] = cv::Mat(pyr_[i-1].rows / 2, pyr_[i-1].cols / 2, CV_8UC1);
      vk::halfSample(pyr_[i-1], pyr_[i]);
    }
  }
  cv::Mat& operator[](int n) const {
    return pyr_[n];
  }
 private:
  cv::Mat *pyr_;
};

static inline int clamp(int x) {
  return max(0, min(255, x));
}

bool Poll(bool block = false) {
  SDL_Event event;

  while (block ? SDL_WaitEvent(&event) : SDL_PollEvent(&event)) {
    switch (event.type) {
      case SDL_KEYDOWN:
        return false;
    }
  }
  return true;
}

void BlitYUVToRGBx2(const uint8_t *yuvbuf, int w, int h, int span,
                  uint32_t *rgbbuf) {
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      int c = yuvbuf[j*w + i] - 16;
      int d = yuvbuf[w*h + (j>>1)*(w>>1) + (i>>1)] - 128;
      int e = yuvbuf[w*(h + (h>>2)) + (j>>1)*(w>>1) + (i>>1)] - 128;
      uint32_t rgb = 0xff000000 |
          (clamp((298*c + 409*e + 128) >> 8)) |
          (clamp((298*c - 100*d - 208*e + 128) >> 8) << 8) |
          (clamp((298*c + 516*d + 128) >> 8) << 16);
      rgbbuf[i*2] = rgb;
      rgbbuf[i*2+1] = rgb;
      rgbbuf[i*2+span] = rgb;
      rgbbuf[i*2+span+1] = rgb;
    }
    rgbbuf += 2*span;
  }
}

void ReverseBuf(uint8_t *buf, int len) {
  for (int i = 0; i < len/2; i++) {
    std::swap(buf[i], buf[len-1-i]);
  }
}

bool ReadFrame(FILE *fp, uint8_t *yuvbuf) {
  const size_t framesize = 320*360;
  uint32_t sec, usec;
  if (fread(&sec, 1, 4, fp) < 4)
    return false;
  if (fread(&usec, 1, 4, fp) < 4)
    return false;
  if (fread(yuvbuf, 1, framesize, fp) < framesize)
    return false;
  // rotate yuvbuf 180 degrees
  ReverseBuf(yuvbuf, 320*240);
  ReverseBuf(yuvbuf+320*240, 160*120);
  ReverseBuf(yuvbuf+320*300, 160*120);
  return true;
}

int main(int argc, char *argv[]) {
  // TODO: specify frame 1 and frame 2, seek when reading input
  if (argc < 2) {
    fprintf(stderr, "%s [output.yuv]\n", argv[0]);
    return 1;
  }

  FILE *fp = NULL;
  if (!strcmp(argv[1], "-")) {
    fp = stdin;
    setbuf(stdin, NULL);
  } else {
    fp = fopen(argv[1], "rb");
  }
  if (!fp) {
    perror(argv[1]);
    return 1;
  }

  SDL_Init(SDL_INIT_VIDEO);

  SDL_Surface *screen = SDL_SetVideoMode(1280, 480, 32, SDL_SWSURFACE);
  if (!screen) {
    fprintf(stderr, "sdl screen init fail\n");
    return 1;
  }
  SDL_WM_SetCaption("keypoint tracking test", NULL);

  SDL_Surface *frame = SDL_CreateRGBSurface(
      SDL_SWSURFACE, 1280, 480, 32,
      0x000000ff, 0x0000ff00, 0x00ff0000, 0xff000000);

  int frameno = 0;
  ImagePyramid framepyr[2];
  uint8_t framebuf[2][320*360];
  for (int i = 0; i < 2; i++) {
    if (!ReadFrame(fp, framebuf[i]))
      return 1;
    framepyr[i].Init(framebuf[i], n_pyr_levels);
  }

  SDL_LockSurface(frame);
  uint32_t *pixbuf = reinterpret_cast<uint32_t*>(frame->pixels);
  BlitYUVToRGBx2(framebuf[0], 320, 240, 1280, pixbuf);
  BlitYUVToRGBx2(framebuf[1], 320, 240, 1280, pixbuf + 640);

  {
    ImagePyramid& img_pyr = framepyr[0];
    const int L = 1;
    vector<fast::fast_xy> fast_corners;
    const int threshold = 20;
    fast::fast_corner_detect_10(
        (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
        img_pyr[L].rows, img_pyr[L].cols, threshold, fast_corners);
    vector<int> scores, nm_corners;
    fast::fast_corner_score_10(
        (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
        fast_corners, threshold, scores);
    fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

    vector<cv::Point2f> corners_f0, corners_f1;
    vector<uint8_t> status_f1;
    vector<float> err;

    for (int i = 0; i < nm_corners.size(); i++) {
      float x = fast_corners[nm_corners[i]].x * (1 << L);
      float y = fast_corners[nm_corners[i]].y * (1 << L);
      corners_f0.push_back(cv::Point2f(x, y));
    }

    // use lucas-kanade to solve for u,v in framepyr[0][x,y] =
    // framepyr[1][u,v]
    calcOpticalFlowPyrLK(framepyr[0][0], framepyr[1][0], corners_f0,
                         corners_f1, status_f1, err, cv::Size(5, 5));

    int found = 0;
    for (int i = 0; i < corners_f0.size(); i++) {
      int x = corners_f0[i].x * 2;
      int y = corners_f0[i].y * 2;
      pixbuf[y * 1280 + x] = 0xff00ff00;
      pixbuf[y * 1280 + x + 1] = 0xff00ff00;
      pixbuf[y * 1280 + x + 1280] = 0xff00ff00;
      pixbuf[y * 1280 + x + 1281] = 0xff00ff00;
      if (!status_f1[i]) continue;
      x = corners_f1[i].x * 2;
      y = corners_f1[i].y * 2;
      pixbuf[640 + y * 1280 + x] = 0xff00ff00;
      pixbuf[640 + y * 1280 + x + 1] = 0xff00ff00;
      pixbuf[640 + y * 1280 + x + 1280] = 0xff00ff00;
      pixbuf[640 + y * 1280 + x + 1281] = 0xff00ff00;
      found++;
    }
    fprintf(stderr, "found %d/%lu corners\n", found, corners_f0.size());
  }
  SDL_UnlockSurface(frame);
  SDL_BlitSurface(frame, NULL, screen, NULL);
  SDL_Flip(screen);

  while (Poll(true)) {}

  return 0;
}
