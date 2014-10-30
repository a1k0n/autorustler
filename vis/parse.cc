#include <stdio.h>
#include <vector>

#include "opencv2/core/core.hpp"
#include "fast/fast.h"
#include "vikit/vision.h"

#include "car/radio.h"
#include "imu/imu.h"
#include "gps/sirf.h"
#include "ui/recording.h"

using std::vector;

void HandleFrame(uint8_t *buf, int w, int h) {
  static cv::Mat prevFrame;
  static vector<cv::Point2f> prevFeatures;
  static bool firstframe = true;

  const int maxlevel = 4;
  const int ws = 4;

  cv::Mat frame = cv::Mat(h, w, CV_8U, buf).clone();

  if (!firstframe && !prevFeatures.empty()) {
    vector<uint8_t> status;
    vector<float> err;
    vector<cv::Point2f> out_points;
    calcOpticalFlowPyrLK(prevFrame, frame, prevFeatures, out_points,
                         status, err, cv::Size(ws, ws), maxlevel);
    int n = 0;
    for (int i = 0; i < status.size(); i++) {
      if (!status[i]) continue;
      n++;
      printf("%f,%f -> %f,%f %f\n", prevFeatures[i].x, prevFeatures[i].y,
             out_points[i].x, out_points[i].y, err[i]);
    }
    printf("%d/%d points tracked\n", n, prevFeatures.size());
  }

  vector<fast::fast_xy> fast_corners;
  int L = 0;  // find corners at bottom of pyramid

  const int scale = 1 << L;
  const int threshold = 40;
  fast_corners.clear();
  fast::fast_corner_detect_10_sse2(buf, w, h, w, threshold, fast_corners);

  vector<int> scores, nm_corners;
  fast::fast_corner_score_10(buf, w, fast_corners, threshold, scores);
  fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);
  prevFeatures.clear();
  for (int i = 0; i < nm_corners.size(); i++) {
    const fast::fast_xy c = fast_corners[nm_corners[i]];
    prevFeatures.push_back(cv::Point2f(c.x, c.y));
  }
  prevFrame = frame;
  firstframe = false;
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
        }
        break;
      case RecordHeader::GPSFrame:
        {
          sirf_navdata gps_state;
          memcpy(&gps_state, buf, sizeof(gps_state));
          printf("gps %d.%06d %d %d %d %d %d %d %d %d\n",
                 rh.ts_sec, rh.ts_usec,
                 gps_state.x, gps_state.y, gps_state.z,
                 gps_state.v8x, gps_state.v8y, gps_state.v8z,
                 gps_state.hdop, gps_state.svs);
        }
    }
  }

  return 0;
}
