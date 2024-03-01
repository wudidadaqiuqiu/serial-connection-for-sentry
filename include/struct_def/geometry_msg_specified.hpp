#pragma once

#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "easy_robot_commands/shared_member/base_caller_template.hpp"

// geometry_msgs::msg::WrenchStamped
namespace EasyRobotCommands {
namespace eamsg = easy_robot_commands::msg;
}

template <>
struct EasyRobotCommands::StructDataT<geometry_msgs::msg::WrenchStamped> {
#pragma pack(1)
    struct data_t {
        float vx;
        float vy;
        float wz;
    };
    data_t data;
    static_assert((sizeof(data_t) == 12));
#pragma pack()
    /* assign operate*/
    StructDataT& operator=(const geometry_msgs::msg::WrenchStamped::SharedPtr msgptr) {
        data.vx = msgptr->wrench.force.x;
        data.vy = msgptr->wrench.force.y;
        data.wz = msgptr->wrench.torque.z;
        return *this;
    }

    static constexpr std::string_view MsgTypeName = "CmdVel";
    static constexpr std::string_view topic_name = "cmd_vel";
    static constexpr uint8_t ID = 0x00;
};