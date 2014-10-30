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
#include "fast/fast.h"
#include "vikit/vision.h"

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

bool calibrated = false;
void CalibrateCamera(const uint8_t *yuvbuf) {
  static int corner_timer = 3;
  static int calib_success = 0;
  const int board_w = 5;
  const int board_h = 8;
  const int board_n = board_w * board_h;
  const int n_boards = 32;
  static CvMat* image_points = cvCreateMat(n_boards * board_n, 2, CV_32FC1);
  static CvMat* object_points = cvCreateMat(n_boards * board_n, 3, CV_32FC1);
  static CvMat* point_counts = cvCreateMat(n_boards, 1, CV_32SC1);

  IplImage *image = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 1);
  cvSetData(image, const_cast<uint8_t*>(yuvbuf), 320);
  CvPoint2D32f corners[board_n];
  int corner_count = 0;
  CvSize board_sz = cvSize(board_w, board_h);
  int found = cvFindChessboardCorners(
      image, board_sz, corners, &corner_count,
      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
  printf("found: %d corner_count: %d\n", found, corner_count);
  cvFindCornerSubPix(
      image, corners, corner_count, cvSize(11, 11),
      cvSize(-1, -1),
      cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
  cvDrawChessboardCorners(image, board_sz, corners, corner_count, found);

  if (corner_count == board_n) {
    corner_timer--;
    if (corner_timer <= 0) {
      corner_timer = 8;
      // add corners to matrix
      printf("adding calibration points %d\n", calib_success);
      int step = calib_success * board_n;
      for (int i = step, j = 0; j < corner_count; i++, j++) {
        CV_MAT_ELEM(*image_points, float, i, 0) = corners[j].x;
        CV_MAT_ELEM(*image_points, float, i, 1) = corners[j].y;
        CV_MAT_ELEM(*object_points, float, i, 0) = j/board_w;
        CV_MAT_ELEM(*object_points, float, i, 1) = j%board_w;
        CV_MAT_ELEM(*object_points, float, i, 2) = 0.0f;
      }
      CV_MAT_ELEM(*point_counts, int, calib_success, 0) = board_n;
      calib_success++;
    }
  }

  if (calib_success == n_boards) {
    CvMat* intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
    CvMat* distortion_coeffs = cvCreateMat(5, 1, CV_32FC1);
    // At this point we have all the chessboard corners we need
    // Initiliazie the intrinsic matrix such that the two focal lengths
    // have a ratio of 1.0

    CV_MAT_ELEM(*intrinsic_matrix, float, 0, 0) = 1.0;
    CV_MAT_ELEM(*intrinsic_matrix, float, 1, 1) = 1.0;

    // Calibrate the camera
    double err = cvCalibrateCamera2(
        object_points, image_points, point_counts,
        cvGetSize(image),
        intrinsic_matrix, distortion_coeffs, NULL, NULL,
        CV_CALIB_FIX_ASPECT_RATIO);

    // Save the intrinsics and distortions
    cvSave("intrinsics.yaml", intrinsic_matrix);
    cvSave("distortion.yaml", distortion_coeffs);
    fprintf(stderr, "calibrated, reconstruction error %f\n", err);

    calibrated = true;
  }
}

// y = dy/dx * x
// y * dx = x * dy
void DrawLine(float x1, float y1, float x2, float y2,
              uint32_t color, int span, uint32_t *buf) {
  if (y1 < 0 || y1 > 479) return;
  if (y2 < 0 || y2 > 479) return;
  if (x1 < 0 || x1 >= span) return;
  if (x2 < 0 || x2 >= span) return;
  float dx = x2 - x1;
  float dy = y2 - y1;
  if (fabs(dy) < fabs(dx)) {
    // x-loop
    if (x2 < x1) {
      std::swap(x1, x2);
      std::swap(y1, y2);
      dx = -dx;
      dy = -dy;
    }
    int y = y1;
    y *= span;
    float accum = 0;
    for (int x = x1; x <= x2; x++) {
      // if (y < 0 || y >= 640*480)
      //   break;
      buf[y + x] = color;
      accum += dy;
      if (accum >= dx) { accum -= dx; y += span; }
      if (accum <= -dx) { accum += dx; y -= span; }
    }
  } else {
    // y-loop
    if (y2 < y1) {
      std::swap(x1, x2);
      std::swap(y1, y2);
      dx = -dx;
      dy = -dy;
    }
    int x = x1;
    int y = y1;
    // FIXME: interpolate to pixel center first
    float accum = 0;
    y *= span;
    int yend = y2;
    yend *= span;
    for (; y <= yend; y += span) {
      buf[y + x] = color;
      accum += dx;
      if (accum >= dy) { accum -= dy; x++; }
      if (accum <= -dy) { accum += dy; x--; }
    }
  }
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

  static cv::Mat prevFrame;
  static vector<cv::Point2f> prevFeatures;
  static bool firstframe = true;
  cv::Mat curFrame = cv::Mat(
      240, 320, CV_8U, const_cast<uint8_t*>(yuvbuf)).clone();

  const int maxlevel = 4;
  const int ws = 8;

  if (!firstframe && !prevFeatures.empty()) {
    vector<uint8_t> status;
    vector<float> err;
    vector<cv::Point2f> out_points;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT +
                              cv::TermCriteria::EPS, 30, 0.01);
    calcOpticalFlowPyrLK(prevFrame, curFrame, prevFeatures, out_points,
                         status, err, cv::Size(ws, ws), maxlevel,
                         termcrit, cv::OPTFLOW_LK_GET_MIN_EIGENVALS);
    for (int i = 0; i < status.size(); i++) {
      if (!status[i]) continue;
      DrawLine(640 - prevFeatures[i].x * 2, 480 - prevFeatures[i].y * 2,
               640 - out_points[i].x * 2, 480 - out_points[i].y * 2,
               0xff00cc00, 640, pixbuf);
    }
  }

  const int threshold = 40;
  vector<fast::fast_xy> fast_corners;
#if __SSE2__
  fast::fast_corner_detect_10_sse2(
      yuvbuf, 320, 240, 320, threshold, fast_corners);
#else
  fast::fast_corner_detect_10(
      yuvbuf, 320, 240, 320, threshold, fast_corners);
#endif

  vector<int> scores, nm_corners;
  fast::fast_corner_score_10(yuvbuf, 320, fast_corners, threshold, scores);
  fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

  prevFeatures.clear();
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
  prevFrame = curFrame;
  firstframe = false;

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
