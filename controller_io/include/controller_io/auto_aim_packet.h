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
    uint8_t id;
    uint8_t delay;
    uint8_t is_tracking;
    uint8_t is_follow;
};

struct GimbalPosePacket {
    uint64_t timestamp;
    float w, x, y, z;
    uint16_t delay;
    float px, py, pz;
};
}
#endif
