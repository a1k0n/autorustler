#ifndef GPS_UBX_H_
#define GPS_UBX_H_

#include <stdint.h>

// Earth-Centered, Earth-Fixed position message from GPS
struct nav_posecef {
  uint32_t iTOW;  // millisecond time of week
  int32_t ecefX, ecefY, ecefZ;  // ECEF position in cm
  uint32_t pAcc;  // position accuracy estimate
};

int ubx_open();
void ubx_read_loop(int fd, void (*on_ecef)(const nav_posecef*));

#endif  // GPS_UBX_H_
