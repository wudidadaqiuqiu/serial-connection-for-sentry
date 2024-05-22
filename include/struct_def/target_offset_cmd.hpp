#pragma once

// #include "geometry_msgs/msg/wrench_stamped.hpp"
#include "robot_msgs/msg/cam_command.hpp"
#include "easy_robot_commands/shared_member/base_caller_template.hpp"
#include "math.h"
#include <iostream>
// geometry_msgs::msg::WrenchStamped
namespace EasyRobotCommands {
namespace eamsg = easy_robot_commands::msg;
}

template <>
struct EasyRobotCommands::StructDataT<robot_msgs::msg::CamCommand> {
#pragma pack(1)
    enum auto_shoot_mode_e : uint8_t{
        auto_aim_off = 0, 
        auto_aim_normal, // 普通自瞄
        auto_aim_outpost,
        auto_aim_outpost_up,
        auto_aim_buff_small,// 小符
        auto_aim_buff_big,//大符
        // auto_aim_anti_buff,//反符
        autoaim_stop,
        autoaim_unkonw,
        auto_aim_mode_num,
    };

    struct data_t {
        int16_t ref_offset;
        uint8_t pitch_mode;
        uint8_t autoaim_rate;
        uint8_t priority_type_arr[8];
        uint8_t priority_level_arr[8];
        auto_shoot_mode_e autoaim_mode;
    };
    data_t data;
    static_assert((sizeof(data_t) == 21));
#pragma pack()
    /* assign operate*/
    StructDataT& operator=(const robot_msgs::msg::CamCommand::SharedPtr msgptr) {
        if (fabs(msgptr->yaw) > 180) {
            data.ref_offset = 360;
        } else {
            data.ref_offset = msgptr->yaw;
        }

        data.pitch_mode = msgptr->pitch_mode;
        data.autoaim_rate = msgptr->autoshoot_rate;

        try {
            for (int i = 0; i < 8; ++i) {
                data.priority_type_arr[i] = msgptr->priority_type_arr.at(i);
                data.priority_level_arr[i] = msgptr->priority_level_arr.at(i);
            }
            *(uint8_t*)&data.autoaim_mode = msgptr->autoaim_mode;
        } catch (std::exception& e) {
            std::cerr << "!!!!!!!!!!!!!!!!        arr must be 8 len     !!!!!!!!!!!!!!" << std::endl;
            std::cerr << "\n\n";
            std::cerr << "throw by serial_connection " << __FILE__ << " " << __LINE__ << std::endl;
            std::cerr << "\n\n";
            throw;
        }

        return *this;
    }

    static constexpr std::string_view MsgTypeName = "CamCommand";
    static constexpr std::string_view topic_name = "easy_robot_commands/cam_command";
    static constexpr uint8_t ID = 0x01;
};