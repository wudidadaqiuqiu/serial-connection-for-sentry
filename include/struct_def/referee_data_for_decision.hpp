#pragma once
#include <cstdint>
#include "robot_msgs/msg/referee_data.hpp"
#include "robot_msgs/msg/robot_battle_state.hpp"
#include <cstring>

namespace StructDef {
typedef enum Robot_id_e : uint8_t{
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_SENTRY = 7,
    RED_OUTPOST = 8,
    RED_BASE = 9,
    BLUE_HERO = 101,
    BLUE_ENGINEER = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_SENTRY = 107,
    BLUE_OUTPOST = 108,
    BLUE_BASE = 109,
} Robot_id;


struct referee_data_for_decision {
    #pragma pack(1)
    typedef struct referee_data_for_desision_t {
        uint16_t team_HP[16];
        int16_t game_time;
        uint8_t robot_id;
        uint16_t allow_bullet;
    } referee_data_for_desision_t;
    #pragma pack()
    referee_data_for_desision_t data;
    static constexpr uint8_t ID = 0x11;
    static constexpr std::string_view sub_topic = "referee_data_for_decision";
    template <typename T>
    void transfer_to(T& msg);
};

template <>
void referee_data_for_decision::transfer_to<robot_msgs::msg::RefereeData>(robot_msgs::msg::RefereeData& msg) {
    msg.allow_bullet = data.allow_bullet;
    msg.game_time = data.game_time;
    msg.robot_id = data.robot_id;
    msg.data.resize(16);
    msg.data.at(0).id = Robot_id::RED_HERO;
    msg.data.at(1).id = Robot_id::RED_ENGINEER;
    msg.data.at(2).id = Robot_id::RED_STANDARD_1;
    msg.data.at(3).id = Robot_id::RED_STANDARD_2;
    msg.data.at(4).id = Robot_id::RED_STANDARD_3;
    msg.data.at(5).id = Robot_id::RED_SENTRY;
    msg.data.at(6).id = Robot_id::RED_OUTPOST;
    msg.data.at(7).id = Robot_id::RED_BASE;
    msg.data.at(8).id = Robot_id::BLUE_HERO;
    msg.data.at(9).id = Robot_id::BLUE_ENGINEER; 
    msg.data.at(10).id = Robot_id::BLUE_STANDARD_1;
    msg.data.at(11).id = Robot_id::BLUE_STANDARD_2;
    msg.data.at(12).id = Robot_id::BLUE_STANDARD_3;
    msg.data.at(13).id = Robot_id::BLUE_SENTRY; 
    msg.data.at(14).id = Robot_id::BLUE_OUTPOST;
    msg.data.at(15).id = Robot_id::BLUE_BASE;
    for (int i = 0; i < 16; ++i) {
        msg.data.at(i).blood = data.team_HP[i];
    }
}
}