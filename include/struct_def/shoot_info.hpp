#pragma once
#include <cstdint>
#include "easy_robot_commands/msg/robot_shoot_info.hpp"
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
    static constexpr std::string_view sub_topic = "robot_shoot_info";
    template <typename T>
    void transfer_to(T& msg);
};

template <>
void shoot_info_t::transfer_to<eamsg::RobotShootInfo>(eamsg::RobotShootInfo& msg) {
    msg.rpy[0] = data.euler[0];
    msg.rpy[1] = data.euler[1];
    msg.rpy[2] = data.euler[2];
    msg.mode = static_cast<uint8_t>(data.auto_mode_flag);
    msg.robot_id = data.robot_id;
    msg.x = data.bullet_speed_0_to_127 | data.is_shoot_data_updated;
}
}