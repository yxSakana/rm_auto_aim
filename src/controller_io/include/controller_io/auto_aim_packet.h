#ifndef ARMOR_AUTO_AIM_AUTO_AIM_PACKET_H
#define ARMOR_AUTO_AIM_AUTO_AIM_PACKET_H

#include <cstdint>

namespace armor_auto_aim {
struct AutoAimPacket {
    float x, y, z,
          v_x, v_y, v_z,
          theta, omega,
          r1, r2, dz;
    uint8_t num;
    uint8_t delay;
    uint8_t is_tracking;
};

struct GimbalPosePacket {
    uint64_t timestamp;
    float w, x, y, z;
    float yaw, pitch, roll;
};
}
#endif
