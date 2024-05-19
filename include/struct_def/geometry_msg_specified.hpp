#pragma once

#include "robot_msgs/msg/walk_cmd.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "easy_robot_commands/shared_member/base_caller_template.hpp"

// geometry_msgs::msg::WrenchStamped
namespace EasyRobotCommands {
namespace eamsg = easy_robot_commands::msg;
}

template <>
struct EasyRobotCommands::StructDataT<geometry_msgs::msg::WrenchStamped> {
#pragma pack(1)
    typedef enum cap_mode_e : uint8_t {
        CAP_NORMAL = 0,
        CAP_HIGH = 1,
        CAP_LOW = 2,
    } cap_mode_e;
    struct data_t {
        float vx;
        float vy;
        float wz;
        cap_mode_e cap_mode;
    };
    data_t data;
    static_assert((sizeof(data_t) == 13));
#pragma pack()
    /* assign operate*/
    StructDataT& operator=(const geometry_msgs::msg::WrenchStamped::SharedPtr msgptr) {
        data.vx = msgptr->wrench.force.x;
        data.vy = msgptr->wrench.force.y;
        data.wz = msgptr->wrench.torque.z;
        // std::cout << "subscribe /cmd_vel" << std::endl;
        return *this;
    }

    StructDataT& operator=(const robot_msgs::msg::WalkCmd::SharedPtr msgptr) {
        *(uint8_t*)&data.cap_mode = msgptr->cap_mode;
        return *this;
    }

    static constexpr std::string_view MsgTypeName = "CmdVel";
    static constexpr std::string_view topic_name = "cmd_vel";
    static constexpr uint8_t ID = 0x00;
};

// static inline void walkcmd_to_cmdvel_caller(const robot_msgs::msg::WalkCmd::SharedPtr walkcmd, 
//         ea_base_caller<geometry_msgs::msg::WrenchStamped>& caller) {
    
// }