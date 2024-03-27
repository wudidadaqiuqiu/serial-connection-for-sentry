#pragma once
#include <cstdint>
// #include <string>
#include <cstring>
#include "robot_msgs/msg/referee_game_state.hpp"

namespace StructDef {

struct referee_game_state {
    typedef enum : uint8_t{
        RMUC = 1,
        RMUL = 4,
    } game_type_e;
    typedef enum : uint8_t{
        PROGRESS_UNSTART = 0,
        PROGRESS_PREPARE = 1,
        PROGRESS_SELFCHECK = 2,
        PROGRESS_5sCOUNTDOWN = 3,
        PROGRESS_BATTLE = 4,
        PROGRESS_CALCULATING = 5,
    } game_progress_e;

    #pragma pack(1)
    typedef struct referee_game_state_t {
        game_type_e game_type;
        game_progress_e game_progress;
    } referee_game_state_t;
    #pragma pack()
    referee_game_state_t data;
    static constexpr uint8_t ID = 0x13;
    static constexpr std::string_view sub_topic = "referee_game_state";
    template <typename T>
    void transfer_to(T& msg);
};

template <>
void referee_game_state::transfer_to<robot_msgs::msg::RefereeGameState>(robot_msgs::msg::RefereeGameState& msg) {
    msg.game_type = data.game_type;
    msg.game_progress = data.game_progress;
}
}