#ifndef CAR_LAMP_TRACK_TYPES_H_
#define CAR_LAMP_TRACK_TYPES_H_

#include "zf_common_typedef.h"

/* Fixed 32-byte authority snapshot carried in each Camera SPI control frame. */
typedef struct
{
    uint32 sequence;
    uint32 reference_time_ms;
    uint32 last_support_time_ms;
    float center_x;
    float center_y;
    float velocity_x;
    float velocity_y;
    uint8 state;
    uint8 support_camera_mask;
    uint8 quality;
    uint8 roi_mode;
} car_lamp_track_snapshot_t;

typedef char car_lamp_track_snapshot_size_must_be_32[
    (sizeof(car_lamp_track_snapshot_t) == 32U) ? 1 : -1];

#endif /* CAR_LAMP_TRACK_TYPES_H_ */
