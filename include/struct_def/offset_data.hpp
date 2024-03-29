#pragma once
#include <cstdint>
// #include <string>
#include <cstring>
#include "robot_msgs/msg/gimbal_data.hpp"
#include "referee_data_for_decision.hpp"
namespace StructDef {
struct robot_offset_data {
    #pragma pack(1)
    typedef struct offset_data_t {
        int16_t offset;
        uint8_t virtual_mode;
    } offset_data_t;
    #pragma pack()
    offset_data_t data;
    static constexpr uint8_t ID = 0x12;
    static constexpr std::string_view sub_topic = "offset_data";
    template <typename T>
    void transfer_to(T& msg);
};

template <>
void robot_offset_data::transfer_to<robot_msgs::msg::GimbalData>(robot_msgs::msg::GimbalData& msg) {
    msg.offset = data.offset;
    msg.virtual_mode = data.virtual_mode;
}
}