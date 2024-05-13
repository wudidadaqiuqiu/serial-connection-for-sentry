#pragma once

// #include "geometry_msgs/msg/wrench_stamped.hpp"
#include "robot_msgs/msg/target_offset.hpp"
#include "easy_robot_commands/shared_member/base_caller_template.hpp"
#include "math.h"
#include <iostream>
// geometry_msgs::msg::WrenchStamped
namespace EasyRobotCommands {
namespace eamsg = easy_robot_commands::msg;
}

template <>
struct EasyRobotCommands::StructDataT<robot_msgs::msg::TargetOffset> {
#pragma pack(1)
    struct data_t {
        int16_t ref_offset;
        // float vx;
        // float vy;
        // float wz;
        
    };
    data_t data;
    static_assert((sizeof(data_t) == 2));
#pragma pack()
    /* assign operate*/
    StructDataT& operator=(const robot_msgs::msg::TargetOffset::SharedPtr msgptr) {
        if (fabs(msgptr->target_offset) > 180) {
            std::cout << "target offset float out of range(-180, 180)" << std::endl;
        } else {
            data.ref_offset = msgptr->target_offset;
        }
        return *this;
    }

    static constexpr std::string_view MsgTypeName = "TargetOffset";
    static constexpr std::string_view topic_name = "easy_robot_commands/target_offset";
    static constexpr uint8_t ID = 0x01;
};