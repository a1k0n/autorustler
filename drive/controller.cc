#include <stdint.h>

#include "drive/controller.h"
#include "drive/udplane.h"

// TODO: remove this
static const float CONST_P = 1.0 / 500.0;
static const float CONST_V = 700;  // safe speed: 700

void DriveController::UpdateState(uint8_t *yuv, size_t yuvlen,
      uint16_t throttle_in, uint16_t steering_in,
      const Eigen::Vector3f &accel,
      const Eigen::Vector3f &gyro) {


  // linear regression on yellow pixels in undistorted/reprojected ground plane
  float sumxy = 0, sumx = 0, sumy = 0, sumy2 = 0, n = 0;

  // size_t ybufidx = 0;  // notused
  size_t bufidx = udplane_ytop*320;
  // size_t vbufidx = 640*480 + 320*240 + udplane_ytop*320;  // notused yet

  for (int y = 0; y < 240 - udplane_ytop; y++) {
    for (int x = 0; x < 320; x++, bufidx++) {
      uint8_t u = yuv[640*480 + bufidx];
      // uint8_t v = yuv[640*480 + 320*240 + bufidx];
      if (u >= 90) continue;  // was 105
      // if (v <= 140) continue;
      
      float pu = udplane[bufidx*2];
      float pv = udplane[bufidx*2 + 1];

      // add x, y to linear regression
      sumx += pu;
      sumy += pv;
      sumxy += pu*pv;
      sumy2 += pv*pv;
      n += 1;
    }
  }

  n_ = n;
  if (n > 5) {
    // intercept is the projection of the line on the bottom of the screen
    // which is our steering error
    // the slope also determines our angle offset
    sumx /= n;
    sumy /= n;
    sumxy /= n;
    sumy2 /= n;
    beta_ = (sumxy - sumx*sumy) / (sumy2 - sumy*sumy);
    alpha_ = sumx - beta_*sumy;
    last_side_ = alpha_ > 160 ? 1 : -1;
  }
}

bool DriveController::GetControl(uint16_t *steer_out, uint16_t *throttle_out) {
  if (n_ > 5) {
    float s = CONST_P * (alpha_ - 160);
    float s0 = -6500 / 32767.0;
    *steer_out = 614.4 - 204.8*(s0 + s);
    *throttle_out = CONST_V;
  } else {
    *steer_out = 614.4 - 204.8 * last_side_;
    *throttle_out = CONST_V - 20;
  }
  return true;
}

