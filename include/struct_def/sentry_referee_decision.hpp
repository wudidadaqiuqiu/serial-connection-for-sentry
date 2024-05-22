#pragma once

// #include "geometry_msgs/msg/wrench_stamped.hpp"
// #include "robot_msgs/msg/sentry_referee_decision.hpp"
#include "robot_msgs/msg/sentry_referee_decision.hpp"
#include "easy_robot_commands/shared_member/base_caller_template.hpp"
#include "math.h"
#include <iostream>
// geometry_msgs::msg::WrenchStamped
template <>
struct EasyRobotCommands::StructDataT<robot_msgs::msg::SentryRefereeDecision> {
#pragma pack(1)
    struct data_t {
        uint32_t decide_to_resurrected : 1;
        uint32_t decide_to_buy_life : 1;
        uint32_t bullet_num_to_buy : 11;
        uint32_t requst_remote_buy_bullet_num : 4;
        uint32_t requst_remote_buy_blood_num : 4;
        uint32_t reserved_bits : 11;
    } ;
    data_t data;
    static_assert((sizeof(data_t) == 4));
#pragma pack()
    /* assign operate*/
    StructDataT& operator=(const robot_msgs::msg::SentryRefereeDecision::SharedPtr msgptr) {
        data.decide_to_resurrected = msgptr->decide_to_resurrected;
        data.bullet_num_to_buy = msgptr->bullet_num_to_buy;
        data.decide_to_buy_life = msgptr->decide_to_buy_life;
        data.requst_remote_buy_blood_num = msgptr->requst_remote_buy_blood_num;
        data.requst_remote_buy_bullet_num = msgptr->requst_remote_buy_bullet_num;
        return *this;
    }

    static constexpr std::string_view MsgTypeName = "SentryRefereeDecision";
    static constexpr std::string_view topic_name = "easy_robot_commands/sentry_referee_decision";
    static constexpr uint8_t ID = 0x05;
};