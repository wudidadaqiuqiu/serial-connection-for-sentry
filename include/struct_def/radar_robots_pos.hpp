#pragma once
#include <cstdint>
// #include <string>
#include <cstring>
#include "robot_msgs/msg/radar_info.hpp"

namespace StructDef {
struct radar_robots_pos {
    #pragma pack(1)
    typedef struct robot_pos_t {
        uint8_t id;
        float x;
        float y;
    } robot_pos_t;

    typedef struct radar_identify_robots_pos_t {
        uint8_t arr_len;
        robot_pos_t robot_pos_arr[12];
    } radar_identify_robots_pos_t;
    #pragma pack()
    radar_identify_robots_pos_t data;
    static constexpr uint8_t ID = 0x15;
    static constexpr std::string_view sub_topic = "radar_info";
    template <typename T>
    void transfer_to(T& msg);
};

template <>
void radar_robots_pos::transfer_to<robot_msgs::msg::RadarInfo>(robot_msgs::msg::RadarInfo& msg) {
    msg.data.data.resize(data.arr_len);
    for (int i = 0; i < data.arr_len; ++i) {
        msg.data.data.at(i).id = data.robot_pos_arr[i].id;
        msg.data.data.at(i).pos.x = data.robot_pos_arr[i].x;
        msg.data.data.at(i).pos.y = data.robot_pos_arr[i].y;
    }
}

}