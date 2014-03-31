#ifndef GPS_SIRF_H_
#define GPS_SIRF_H_

#include <stdint.h>

struct sirf_navdata {
  int32_t x, y, z;  // ECEF xyz in meters
  int16_t v8x, v8y, v8z;  // velocity in 1/8 m/s
  uint8_t hdop;  // dilution of precision in 1/5
  uint8_t svs;  // number of SVs in fix
  uint16_t gps_week;
  uint32_t gps_tow;  // time of week
};

int sirf_open();
bool sirf_poll(int fd, void (*navpos_cb)(const sirf_navdata&));

#endif  // GPS_SIRF_H_
