#ifndef DRIVE_UDPLANE_H_
#define DRIVE_UDPLANE_H_

#include <stdint.h>

static const int udplane_ytop = 100;
extern const float udplane[320*(240 - udplane_ytop)*2];
extern const uint8_t udmask[320*(240 - udplane_ytop)];

#endif  // DRIVE_UDPLANE_H_
