#pragma once

// #include "geometry_msgs/msg/wrench_stamped.hpp"
#include "robot_msgs/msg/decision_points.hpp"
#include "easy_robot_commands/shared_member/base_caller_template.hpp"

// geometry_msgs::msg::WrenchStamped
namespace EasyRobotCommands {
namespace eamsg = easy_robot_commands::msg;
}

template <>
struct EasyRobotCommands::StructDataT<robot_msgs::msg::DecisionPoints> {
#pragma pack(1)
    struct data_t {
        uint8_t intension;
        uint16_t start[2];
        int8_t deltax[2];
        int8_t deltay[2];
    };
    data_t data;
    static_assert((sizeof(data_t) == 9));
#pragma pack()
    /* assign operate*/
    StructDataT& operator=(const robot_msgs::msg::DecisionPoints::SharedPtr msgptr) {
        data.intension = msgptr->intention;
        data.start[0] = msgptr->start.x * 10;
        data.start[1] = msgptr->start.y * 10;
        // 取最大值
#define MAX(a, b) (a > b ? a : b)
// 取最小值
#define MIN(a, b) (a < b ? a : b)
// 双端限幅 a<x<b
#define MID(x, a, b) MAX(MIN(x, b), a)

        data.deltax[0] = MID((msgptr->point1.x - msgptr->start.x) * 10, -128, 127);
        data.deltay[0] = MID((msgptr->point1.y - msgptr->start.y) * 10, -128, 127);

        data.deltax[1] = MID((msgptr->point2.x - msgptr->point1.x) * 10, -128, 127);
        data.deltay[1] = MID((msgptr->point2.y - msgptr->point1.y) * 10, -128, 127);

        return *this;
    }

    static constexpr std::string_view MsgTypeName = "DecisioPoints";
    static constexpr std::string_view topic_name = "easy_robot_commands/decision_points";
    static constexpr uint8_t ID = 0x02;
};