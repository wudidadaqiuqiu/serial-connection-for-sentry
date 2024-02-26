#pragma once
#include <cstdint>

namespace StructDef {
enum class auto_shoot_mode_e : uint8_t {
    auto_aim_off = 0,
    auto_aim_normal,      // 普通自瞄
    auto_aim_buff_small,  // 小符
    auto_aim_buff_big,    // 大符
    auto_aim_anti_buff,   // 反符
    auto_aim_mode_num,
};

struct shoot_info_t {
    #pragma pack(1)
    typedef struct shoot_info_to_pc_t {
        float euler[3];
        auto_shoot_mode_e auto_mode_flag;  // 自瞄模式
        uint8_t robot_id;                  // 当前机器人ID

        uint8_t is_shoot_data_updated : 1;
        // (25, 32)
        uint8_t bullet_speed_0_to_127 : 7;
    } shoot_info_to_pc_t;
    #pragma pack()
    shoot_info_to_pc_t data;
    static constexpr uint8_t ID = 0x03;
};
}