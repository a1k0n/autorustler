#include <assert.h>
#include <stdio.h>
#include <algorithm>
#include <vector>

#include "SDL/SDL.h"
#include "SDL/SDL_ttf.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/features2d/features2d.hpp"

#include "car/radio.h"
#include "imu/imu.h"
#include "gps/sirf.h"
#include "ui/recording.h"

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

IMURawState imu_state;
RCState rc_state;
sirf_navdata gps_state;

void RenderFrame(uint32_t sec, uint32_t usec,
                 const uint8_t *yuvbuf, SDL_Surface *frame) {
  uint32_t *pixbuf = reinterpret_cast<uint32_t*>(frame->pixels);
  for (int j = 0; j < 240; j++) {
    // the camera is rotated 180 degrees, so render it upside-down
    // (bottom-to-top, right-to-left)
    int idx = (480 - j*2) * 640;
    for (int i = 0; i < 320; i++) {
      int c = yuvbuf[j*320 + i] - 16;
      int d = yuvbuf[320*240 + (j>>1)*160 + (i>>1)] - 128;
      int e = yuvbuf[320*300 + (j>>1)*160 + (i>>1)] - 128;
      uint32_t rgb = 0xff000000 |
          (clamp((298*c + 409*e + 128) >> 8)) |
          (clamp((298*c - 100*d - 208*e + 128) >> 8) << 8) |
          (clamp((298*c + 516*d + 128) >> 8) << 16);
      pixbuf[idx-1] = rgb;
      pixbuf[idx-2] = rgb;
      pixbuf[idx-641] = rgb;
      pixbuf[idx-642] = rgb;
      idx -= 2;
    }
  }

  cv::Mat curFrame = cv::Mat(
      240, 320, CV_8U, const_cast<uint8_t*>(yuvbuf)).clone();

#if 0
  for (int i = 0; i < nm_corners.size(); i++) {
    const fast::fast_xy c = fast_corners[nm_corners[i]];
    prevFeatures.push_back(cv::Point2f(c.x, c.y));

    int x = c.x * 2;
    int y = c.y * 2;
    pixbuf[(480 - y) * 640 - x - 1] = 0xff00ff00;
    pixbuf[(480 - y) * 640 - x - 2] = 0xff00ff00;
    pixbuf[(480 - y) * 640 - x] = 0xff00ff00;
    pixbuf[(479 - y) * 640 - x - 1] = 0xff00ff00;
    pixbuf[(481 - y) * 640 - x - 1] = 0xff00ff00;
  }
#endif

  // draw indicators on the screen for various things
  // "zero" is 116
  for (int y = 0; y < 8; y++) {
    if (rc_state.throttle < 116) {
      for (int x = rc_state.throttle; x <= 116; x++) {
        pixbuf[y*640 + x] = 0xff0000ff;  // red: brake
      }
    } else {
      for (int x = 115; x <= rc_state.throttle; x++) {
        pixbuf[y*640 + x] = 0xff00ff00;  // green: throttle
      }
    }
    // black|white: steering
    pixbuf[(y+8)*640 + rc_state.steering] = 0xff000000;
    pixbuf[(y+8)*640 + rc_state.steering+1] = 0xffffffff;
    // red: gyro
    int gx = std::min(638, std::max(0, 116 + (imu_state.gyro_x / 40)));
    pixbuf[(y+16)*640 + gx] = 0xff0000ff;
    pixbuf[(y+16)*640 + gx+1] = 0xff0000ff;
    int gy = std::min(638, std::max(0, 116 + (imu_state.gyro_y / 40)));
    pixbuf[(y+24)*640 + gy] = 0xff0000ff;
    pixbuf[(y+24)*640 + gy+1] = 0xff0000ff;
    int gz = std::min(638, std::max(0, 116 + (imu_state.gyro_z / 40)));
    pixbuf[(y+32)*640 + gz] = 0xff0000ff;
    pixbuf[(y+32)*640 + gz+1] = 0xff0000ff;
  }

  // blue: accel
  int ax = std::min(638, std::max(0, 116 + (-imu_state.accel_y / 16)));
  int ay = std::min(400, std::max(0, 100 + (imu_state.accel_x / 16)));
  pixbuf[(40+100)*640 + 116] = 0xff000000;
  pixbuf[(40+100)*640 + 116+1] = 0xff000000;
  pixbuf[(40+1+100)*640 + 116] = 0xff000000;
  pixbuf[(40+1+100)*640 + 116+1] = 0xff000000;
  DrawLine(ax+0.5, 40+ay+0.5, 116.5, 140.5, 0xff0000ff, 640, pixbuf);
  pixbuf[(40+ay)*640 + ax] = 0xff0000ff;
  pixbuf[(40+ay)*640 + ax+1] = 0xff0000ff;
  pixbuf[(40+1+ay)*640 + ax] = 0xff0000ff;
  pixbuf[(40+1+ay)*640 + ax+1] = 0xff0000ff;
}

int main(int argc, char *argv[]) {
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

  SDL_Surface *screen = SDL_SetVideoMode(640, 480, 32, SDL_SWSURFACE);
  if (!screen) {
    fprintf(stderr, "sdl screen init fail\n");
    return 1;
  }
  SDL_WM_SetCaption("autorustler viz", NULL);

  SDL_Surface *frame = SDL_CreateRGBSurface(
      SDL_SWSURFACE, 640, 480, 32,
      0x000000ff, 0x0000ff00, 0x00ff0000, 0xff000000);

  memset(&imu_state, 0, sizeof(imu_state));
  memset(&rc_state, 0, sizeof(rc_state));
  memset(&gps_state, 0, sizeof(gps_state));
  for (;;) {
    uint32_t sec, usec;
    uint8_t yuvbuf[320*360];
    RecordHeader rh;
    if (fread(&rh, 1, sizeof(rh), fp) < sizeof(rh))
      break;
    if (rh.len > sizeof(yuvbuf)) {
      fprintf(stderr, "unexpectedly large record: type %d len %d\n",
              rh.recordtype, rh.len);
      break;
    }
    if (fread(yuvbuf, 1, rh.len, fp) != rh.len) {
      fprintf(stderr, "truncated record\n");
      break;
    }
    switch (rh.recordtype) {
      case RecordHeader::VideoFrame:
        {
          SDL_LockSurface(frame);
          // FIXME: render latest IMU/GPS state to framebuf
          RenderFrame(sec, usec, yuvbuf, frame);
          SDL_UnlockSurface(frame);
          SDL_BlitSurface(frame, NULL, screen, NULL);
          SDL_Flip(screen);
          if (!Poll())
            return 0;
          SDL_Delay(50);
        }
        break;
      case RecordHeader::IMUFrame:
        {
          // really, i should have a 10Hz bandwidth filter here as IMU is
          // sampled at 50Hz
          memcpy(&imu_state, yuvbuf, sizeof(imu_state));
          memcpy(&rc_state, yuvbuf + sizeof(imu_state), sizeof(rc_state));
#if 0
          fprintf(stderr, "imu [%5d %5d %5d] [%5d %5d %5d] %d %d\n",
                  imu_state.gyro_x, imu_state.gyro_y, imu_state.gyro_z,
                  imu_state.mag_x, imu_state.mag_y, imu_state.mag_z,
                  rc_state.throttle, rc_state.steering);
#endif
        }
        break;
      case RecordHeader::GPSFrame:
        {
          float mph = sqrt(
              gps_state.v8x*gps_state.v8x +
              gps_state.v8y*gps_state.v8y +
              gps_state.v8z*gps_state.v8z) * 0.279617;

          memcpy(&gps_state, yuvbuf, sizeof(gps_state));
          fprintf(stderr, "%d %d %d %d %d %d %fmph\n",
                  gps_state.x, gps_state.y, gps_state.z,
                  gps_state.v8x, gps_state.v8y, gps_state.v8z, mph);
        }
    }
  }

  return 0;
}
