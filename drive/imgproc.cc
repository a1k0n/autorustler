#include <stdio.h>
#include <stdint.h>
#include <Eigen/Dense>
#include <iostream>

#include "drive/imgproc.h"

using Eigen::Matrix3f;
using Eigen::Vector3f;

static const int ytop = 100;
// uxrange (-56, 55) uyrange (2, 59) x0 -56 y0 2
static const int ux0 = -56, uy0 = 2;

// 30 for home, 15 for diyrobocars shiny track
static const int ACTIV_THRESH = 20;

static const int uxsiz = 111, uysiz = 57;

static const float pixel_scale_m = 0.025;

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
  int32_t accumbuf[uxsiz * uysiz * 3];
  // input is a 640x480 YUV420 image
  memset(accumbuf, 0, uxsiz * uysiz * 3 * sizeof(accumbuf[0]));

  static int snapshot = 0;

  FILE *fp = NULL;
  snapshot++;
  if (snapshot == 40) {
    fp = fopen("snapshot.bin", "w");
  }
  // for each yuv, (maybe) remap into detected
  size_t bufidx = ytop*320;
  size_t udidx = 0;
  for (int j = 0; j < 240 - ytop; j++) {
    for (int i = 0; i < 320; i++, bufidx++, udidx++) {

      uint8_t y = yuv[(j+ytop)*2*640 + 2*i];
      uint8_t u = yuv[640*480 + bufidx];
      uint8_t v = yuv[640*480 + 320*240 + bufidx];

      if (fp) {
        fwrite(&y, 1, 1, fp);
        fwrite(&u, 1, 1, fp);
        fwrite(&v, 1, 1, fp);
      }

      if (!udmask[udidx]) continue;
      int dx = udplane[udidx*2] - ux0;
      int dy = udplane[udidx*2 + 1] - uy0;

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
      if (fp) {
        fwrite(&accumbuf[uidx*3], 4, 3, fp);
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
      if (fp) {
        fwrite(&accumbuf[uidx*3], 4, 3, fp);
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
  Matrix3f regXTX = Matrix3f::Zero();
  Vector3f regXTy = Vector3f::Zero();
  double regyTy = 0;
  int regN = 0;

  for (int j = 0; j < uysiz; j++) {
    for (int i = 0; i < uxsiz-7; i++) {
      int32_t yd =
        -(accumbuf[3*(j*uxsiz + i + 6)] - accumbuf[3*(j*uxsiz + i)])
        + 3*(accumbuf[3*(j*uxsiz + i + 4)] - accumbuf[3*(j*uxsiz + i + 2)]);
      int32_t ud =
        -(accumbuf[3*(j*uxsiz + i + 6) + 1] - accumbuf[3*(j*uxsiz + i) + 1])
        + 3*(accumbuf[3*(j*uxsiz + i + 4) + 1] - accumbuf[3*(j*uxsiz + i + 2) + 1]);
      int32_t vd =
        -(accumbuf[3*(j*uxsiz + i + 6) + 2] - accumbuf[3*(j*uxsiz + i) + 2])
        + 3*(accumbuf[3*(j*uxsiz + i + 4) + 2] - accumbuf[3*(j*uxsiz + i + 2) + 2]);

      // detected = (0.25*hv[:, :, 0] - 2*hv[:, :, 1] + 0.5*hv[:, :, 2] - 30)
      //int32_t detected = (yd >> 2) - (ud << 1) + (vd >> 1) - 60;
      int32_t detected = -ud - ACTIV_THRESH;
      if (fp) {
        fwrite(&yd, 4, 1, fp);
        fwrite(&ud, 4, 1, fp);
        fwrite(&vd, 4, 1, fp);
      }
      if (detected > 0) {
        // add x, y to linear regression
        float pu = pixel_scale_m * (i + ux0 + 3), pv = pixel_scale_m * (j + uy0);
        float w = detected;  // use activation as regression weight
        Vector3f regX(w*pv*pv, w*pv, w);
        regXTX.noalias() += regX * regX.transpose();
        regXTy.noalias() += regX * w * pu;
        regyTy += w * w * pu * pu;
        regN += 1;
      }
    }
  }
  if (fp) {
    fclose(fp);
    fp = NULL;
  }

  std::cout << "activations " << regN << "\n";

  // not enough data, don't even try to do an update
  if (regN < 8) {
    return false;
  }

  Matrix3f XTXinv = regXTX.inverse();
  Vector3f B = XTXinv * regXTy;
  *Bout = B;

  // (XB).T y
  // BT XTy
  float r2 = B.dot(regXTX * B) - 2*B.dot(regXTy) + regyTy;
  // r2 /= regN;

#if 1
  std::cout << "XTX\n" << regXTX << "\n";
  std::cout << "XTy " << regXTy.transpose() << "\n";
  std::cout << "yTy " << regyTy << "\n";
  std::cout << "XTXinv\n" << XTXinv << "\n";
  std::cout << "B " << B.transpose() << "\n";
  std::cout << "r2 " << r2 << "\n";
#endif

  if (isnanf(r2)) {
    return false;
  }

  *Rkout = XTXinv * r2;
  return true;
}
