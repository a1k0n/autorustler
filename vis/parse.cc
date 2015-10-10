#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "flann/flann.hpp"

#include "car/radio.h"
#include "imu/imu.h"
#include "gps/sirf.h"
#include "ui/recording.h"

using std::vector;

cv::BRISK detector_;

struct kp {
  int frame;
  cv::KeyPoint uv;
  uint8_t desc[64];

  kp() {}
  kp(int frame, const cv::KeyPoint& uv, uint8_t* d): frame(frame), uv(uv) {
    memcpy(desc, d, 64);
  }
};

vector<kp> keypoints_;

void HandleFrame(uint8_t *buf, int w, int h) {
  static int frameno = -1;
  frameno++;
  cv::Mat frame = cv::Mat(h, w, CV_8U, buf);
  vector<cv::KeyPoint> keypts;
  cv::Mat descriptors;
  detector_(frame, cv::noArray(), keypts, descriptors);
#if 0
  printf("%lu keypts %d x %d descriptor\n", keypts.size(),
         descriptors.rows, descriptors.cols);
#endif
  for (int i = 0; i < keypts.size(); i++) {
    keypoints_.push_back(kp(frameno, keypts[i], &descriptors.data[64*i]));
#if 0
    printf("%d %f %f ", frameno, keypts[i].pt.x, keypts[i].pt.y);
    for (int j = 0; j < 64; j++)
      printf("%02x", descriptors.data[64*i + j]);
    printf("\n");
#endif
  }
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    fprintf(stderr, "%s [rustlerlog-XXXXXX]\n", argv[0]);
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

  for (;;) {
    uint32_t sec, usec;
    uint8_t buf[320*360];
    RecordHeader rh;
    if (fread(&rh, 1, sizeof(rh), fp) < sizeof(rh))
      break;
    if (rh.len > sizeof(buf)) {
      fprintf(stderr, "unexpectedly large record: type %d len %d\n",
              rh.recordtype, rh.len);
      break;
    }
    if (fread(buf, 1, rh.len, fp) != rh.len) {
      fprintf(stderr, "truncated record\n");
      break;
    }
    switch (rh.recordtype) {
      case RecordHeader::VideoFrame:
        HandleFrame(buf, 320, 240);
        break;
      case RecordHeader::IMUFrame:
        {
#if 1
          IMURawState imu_state;
          RCState rc_state;
          memcpy(&imu_state, buf, sizeof(imu_state));
          memcpy(&rc_state, buf + sizeof(imu_state), sizeof(rc_state));
          printf("imu/rc %d.%06d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                 rh.ts_sec, rh.ts_usec,
                 imu_state.gyro_temp,
                 imu_state.gyro_x,
                 imu_state.gyro_y,
                 imu_state.gyro_z,
                 imu_state.mag_x,
                 imu_state.mag_y,
                 imu_state.mag_z,
                 imu_state.accel_x,
                 imu_state.accel_y,
                 imu_state.accel_z,
                 rc_state.throttle,
                 rc_state.steering);
#endif
        }
        break;
      case RecordHeader::GPSFrame:
        {
#if 0
          sirf_navdata gps_state;
          memcpy(&gps_state, buf, sizeof(gps_state));
          printf("gps %d.%06d %d %d %d %d %d %d %d %d\n",
                 rh.ts_sec, rh.ts_usec,
                 gps_state.x, gps_state.y, gps_state.z,
                 gps_state.v8x, gps_state.v8y, gps_state.v8z,
                 gps_state.hdop, gps_state.svs);
#endif
        }
    }
  }

#if 0
  flann::Logger::setLevel(flann::FLANN_LOG_DEBUG);
  const int nkp = keypoints_.size();
  fprintf(stderr, "# creating flann index with %d keypoints...\n", nkp);
  flann::Matrix<uint8_t> descmatrix(new uint8_t[nkp * 64], nkp, 64);
  for (int i = 0; i < nkp; i++) {
    memcpy(descmatrix[i], keypoints_[i].desc, 64);
    if (i < 50) {
      printf("%d ", i);
      for (int j = 0; j < 64; j++)
        printf("%02x", keypoints_[i].desc[j]);
      printf(" %d %0.0f %0.0f\n",
             keypoints_[i].frame,
             keypoints_[i].uv.pt.x, keypoints_[i].uv.pt.y);
    }
  }
#if 0
  flann::Index<flann::Hamming<uint8_t> >
      ann_idx_(descmatrix, flann::HierarchicalClusteringIndexParams());
  ann_idx_.buildIndex();
  fprintf(stderr, "# saving flann index...\n");
  ann_idx_.save("index.flann");

  int nns = 128;
  vector<vector<size_t> > indices;
  vector<vector<flann::Index<flann::Hamming<uint8_t> >::DistanceType> > dists;
  flann::Matrix<uint8_t> query(keypoints_[4].desc, 1, 64);
  ann_idx_.knnSearch(query, indices, dists, nns, flann::SearchParams(128));
  for (int i = 0; i < indices[0].size(); i++) {
    int k = indices[0][i];
    printf("%d ", k);
    for (int j = 0; j < 64; j++)
      printf("%02x", keypoints_[k].desc[j]);
    printf(" %d %d %0.0f %0.0f\n", dists[0][i],
           keypoints_[k].frame,
           keypoints_[k].uv.pt.x, keypoints_[k].uv.pt.y);
  }
#else
  const int maxcenters = 128;
  flann::Matrix<uint8_t> centers(new uint8_t[maxcenters * 64], maxcenters, 64);
  int ncenters = flann::hierarchicalClustering(
      descmatrix, centers, flann::KMeansIndexParams(),
      flann::Hamming<uint8_t>());
  for (int i = 0; i < ncenters; i++) {
    printf("%d ", i);
    for (int j = 0; j < 64; j++)
      printf("%02x", centers[i][j]);
    printf("\n");
  }
#endif
#endif

  return 0;
}
