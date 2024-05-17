#pragma once
#include <cstdint>
// #include <string>
#include <cstring>
#include "robot_msgs/msg/chassis_info.hpp"
// #include "referee_data_for_decision.hpp"

namespace StructDef {
struct chassis_info_data {
    #pragma pack(1)
    typedef struct chassis_info_t {
        int16_t Vx;
        int16_t Vy;
        int16_t Wz;
        uint16_t cap_v;
    } chassis_info_t;
    #pragma pack()
    chassis_info_t data;
    static constexpr uint8_t ID = 0x16;
    static constexpr std::string_view sub_topic = "chassis_info";
    template <typename T>
    void transfer_to(T& msg);
};

template <>
void chassis_info_data::transfer_to<robot_msgs::msg::ChassisInfo>(robot_msgs::msg::ChassisInfo& msg) {
    msg.vx = data.Vx;
    msg.vy = data.Vy;
    msg.wz = data.Wz;
    msg.cap_voltage = data.cap_v / 10;
}
}