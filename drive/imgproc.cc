#include <stdint.h>
#include <Eigen/Dense>

#include "drive/imgproc.h"

using Eigen::Matrix3f;
using Eigen::Vector3f;

static const int ytop = 100;
// uxrange (-56, 55) uyrange (2, 59) x0 -56 y0 2
static const int ux0 = -56, uy0 = 2;

static const int uxsiz = 111, uysiz = 57;

// (57, 111), (3197, 2)
static const uint16_t bucketcount[uxsiz * uysiz] = {
#include "bucketcount.txt"
};

static const uint16_t floodmap[uxsiz * uysiz] = {
#include "floodmap.txt"
};

static const uint8_t udmask[320*(240-ytop)] = {
#include "udmask.txt"
};

static const int8_t udplane[320*(240-ytop)*2] = {
#include "udplane.txt"
};

bool TophatFilter(const uint8_t *yuv, Vector3f *Bout, Matrix3f *Rkout) {
  static int32_t accumbuf[uxsiz * uysiz * 3];
  // input is a 640x480 YUV420 image
  memset(accumbuf, 0, uxsiz * uysiz * sizeof(accumbuf[0]));

  Matrix3f regXTX = Matrix3f::Zero();
  Vector3f regXTy = Vector3f::Zero();
  double regyTy = 0;
  int regN = 0;

  // for each yuv, (maybe) remap into detected
  size_t bufidx = ytop*320;
  size_t udidx = 0;
  for (int y = 0; y < 240 - ytop; y++) {
    for (int x = 0; x < 320; x++, bufidx++, udidx++) {

      if (!udmask[udidx]) continue;
      int dx = udplane[udidx*2] - ux0;
      int dy = udplane[udidx*2 + 1] - uy0;

      uint8_t y = yuv[bufidx*2];
      uint8_t u = yuv[640*480 + bufidx];
      uint8_t v = yuv[640*480 + 320*240 + bufidx];

      accumbuf[(uxsiz * dy + dx) * 3] += y;
      accumbuf[(uxsiz * dy + dx) * 3 + 1] += u;
      accumbuf[(uxsiz * dy + dx) * 3 + 2] += v;
    }
  }

  size_t uidx = 0;
  // average
  for (int j = 0; j < uysiz; j++) {
    for (int i = 0; i < uxsiz; uidx++, i++) {
      if (bucketcount[uidx] > 0) {
        accumbuf[uidx*3] /= bucketcount[uidx];
        accumbuf[uidx*3 + 1] /= bucketcount[uidx];
        accumbuf[uidx*3 + 2] /= bucketcount[uidx];
      }
    }
  }

  // flood-fill
  uidx = 0;
  for (int j = 0; j < uysiz; j++) {
    for (int i = 0; i < uxsiz; uidx++, i++) {
      if (bucketcount[uidx] == 0) {
        accumbuf[uidx*3] = accumbuf[floodmap[uidx]*3];
        accumbuf[uidx*3 + 1] = accumbuf[floodmap[uidx]*3 + 1];
        accumbuf[uidx*3 + 2] = accumbuf[floodmap[uidx]*3 + 2];
      }
    }
  }

  // horizontal cumsum
  for (int j = 0; j < uysiz; j++) {
    for (int i = 1; i < uxsiz; i++) {
      accumbuf[3*(j*uxsiz + i)] += accumbuf[3*(j*uxsiz + i - 1)];
      accumbuf[3*(j*uxsiz + i) + 1] += accumbuf[3*(j*uxsiz + i - 1) + 1];
      accumbuf[3*(j*uxsiz + i) + 2] += accumbuf[3*(j*uxsiz + i - 1) + 2];
    }
  }

  // horizontal convolution w/ [-1, -1, 2, 2, -1, -1]
  for (int j = 0; j < uysiz; j++) {
    for (int i = 3; i < uxsiz-3; i++) {
      int32_t yd =
        -(accumbuf[3*(j*uxsiz + i + 2)] - accumbuf[3*(j*uxsiz + i - 3)])
        + 3*(accumbuf[3*(j*uxsiz + i + 1)] - accumbuf[3*(j*uxsiz + i)]);
      int32_t ud =
        -(accumbuf[3*(j*uxsiz + i + 2) + 1] - accumbuf[3*(j*uxsiz + i - 3) + 1])
        + 3*(accumbuf[3*(j*uxsiz + i + 1) + 1] - accumbuf[3*(j*uxsiz + i) + 1]);
      int32_t vd =
        -(accumbuf[3*(j*uxsiz + i + 2) + 2] - accumbuf[3*(j*uxsiz + i - 3) + 2])
        + 3*(accumbuf[3*(j*uxsiz + i + 1) + 2] - accumbuf[3*(j*uxsiz + i) + 2]);

      // detected = (0.25*hv[:, :, 0] - 2*hv[:, :, 1] + 0.5*hv[:, :, 2] - 30)
      int32_t detected = (yd >> 2) - (ud << 1) + (vd >> 1) - 30;
      if (detected > 0) {
        // add x, y to linear regression
        float pu = i + ux0, pv = j + uy0;
        float w = detected;  // use activation as regression weight
        Vector3f regX(w*pv*pv, w*pv, w);
        regXTX.noalias() += regX * regX.transpose();
        regXTy.noalias() += regX * w * pu;
        regyTy += w * pu * pu;
        regN += 1;
      }
    }
  }

  // not enough data, don't even try to do an update
  if (regN < 8) {
    return false;
  }

  Matrix3f XTXinv = regXTX.inverse();
  *Bout = XTXinv * regXTy;
  float r2 = Bout->transpose().dot(regXTX * (*Bout)) - 2*(*Bout).dot(regXTy) + regyTy;
  // r2 /= regN;

#if 0
  std::cout << "XTX\n" << regXTX << "\n";
  std::cout << "XTy " << regXTy.transpose() << "\n";
  std::cout << "yTy " << regyTy << "\n";
  std::cout << "XTXinv\n" << XTXinv << "\n";
  std::cout << "B " << B << "\n";
  std::cout << "r2 " << r2 << "\n";
#endif

  *Rkout = XTXinv * r2;
  return true;
}
