#pragma once
#include <cstdint>
// #include <string>
#include <cstring>
#include "robot_msgs/msg/aerial_commands.hpp"

namespace StructDef {
struct robot_aerial_commands {
    #pragma pack(1)
    typedef struct map_command_t{  
    float target_position_x; 
    float target_position_y; 
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint8_t cmd_source;
    }map_command_t;
    #pragma pack()
    map_command_t data;
    static constexpr uint8_t ID = 0x14;
    static constexpr std::string_view sub_topic = "map_command";
    template <typename T>
    void transfer_to(T& msg);
};

template <>
void robot_aerial_commands::transfer_to<robot_msgs::msg::AerialCommands>(robot_msgs::msg::AerialCommands& msg) {
    msg.target_position_x = data.target_position_x;
    msg.target_position_y = data.target_position_y;
    msg.cmd_keyboard = data.cmd_keyboard;
    msg.target_robot_id = data.target_robot_id;
    msg.cmd_source = data.cmd_source;
}

}