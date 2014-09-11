#include <stdio.h>
#include <algorithm>
#include <vector>

#include "SDL/SDL.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/imgproc/imgproc_c.h"


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

void RenderFrame(uint32_t sec, uint32_t usec,
                 const uint8_t *yuvbuf, SDL_Surface *frame) {
#if 0
  if (!calibrated) {
    CalibrateCamera(yuvbuf);
  }
#endif
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


  cv::Mat uimage(240, 320, CV_8UC1, const_cast<uint8_t*>(yuvbuf));
  // construct an opencv Mat
  static vector<cv::KeyPoint> points(100);
  // static cv::FastFeatureDetector fastdet(5);
  points.clear();

  // put a green pixel on each keypoint
  // fastdet.detect(uimage, points);
  // use FAST-9
  static int threshold = 10;
  cv::FASTX(uimage, points, threshold, true,
            cv::FastFeatureDetector::TYPE_9_16);
  if (points.size() > 240) threshold++;
  else if (points.size() < 120 && threshold > 3) threshold--;

  for (size_t i = 0; i < points.size(); i++) {
    int x = points[i].pt.x*2;
    int y = points[i].pt.y*2;
    pixbuf[(480 - y) * 640 - x - 1] = 0xff00ff00;
    pixbuf[(480 - y) * 640 - x - 2] = 0xff00ff00;
    pixbuf[(480 - y) * 640 - x] = 0xff00ff00;
    pixbuf[(479 - y) * 640 - x - 1] = 0xff00ff00;
    pixbuf[(481 - y) * 640 - x - 1] = 0xff00ff00;
  }
  printf("[%d.%06d] threshold %d %lu keypoints\n", sec, usec,
         threshold, points.size());
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
    RenderFrame(sec, usec, yuvbuf, frame);
    SDL_UnlockSurface(frame);
    SDL_BlitSurface(frame, NULL, screen, NULL);
    SDL_Flip(screen);
    if (!Poll())
      break;
    SDL_Delay(50);
  }

  return 0;
}
