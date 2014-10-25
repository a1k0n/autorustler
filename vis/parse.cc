#include <stdio.h>

#include "car/radio.h"
#include "imu/imu.h"
#include "gps/sirf.h"
#include "ui/recording.h"

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
        break;
      case RecordHeader::IMUFrame:
        {
          IMURawState imu_state;
          RCState rc_state;
          memcpy(&imu_state, buf, sizeof(imu_state));
          memcpy(&rc_state, buf + sizeof(imu_state), sizeof(rc_state));
          printf("imu/rc %d.%06d %d %d %d %d %d %d %d %d %d %d\n",
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
                 imu_state.accel_z);
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
