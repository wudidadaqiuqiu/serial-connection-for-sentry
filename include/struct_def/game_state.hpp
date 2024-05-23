#pragma once
#include <cstdint>
// #include <string>
#include <cstring>
#include "robot_msgs/msg/referee_game_state.hpp"

namespace StructDef {

struct referee_game_state {
    #pragma pack(1)
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
    typedef struct event_data_t{
        uint8_t Supply_Station_front_M : 1;
        uint8_t Supply_Station_in_M : 1;
        uint8_t Supply_zone : 1; 
        uint8_t power_rune_spot_status : 1;
        uint8_t small_power_rune_status : 1;
        uint8_t big_power_rune_status : 1;
        uint8_t Circular_highland_M : 2;
        uint8_t trapezium_highland_3_M : 2;
        uint8_t trapezium_highland_4_M : 2;
        uint8_t Virtual_Shield : 7;
        uint16_t ennmy_dart_last_time: 9; 
        uint8_t ennmy_dart_info : 2; 
        uint8_t rfid_core_RMUL : 2; 
    }event_data_t;
    typedef struct { 
        float hero_x;  
        float hero_y;  
        float engineer_x;  
        float engineer_y;  
        float standard_3_x;  
        float standard_3_y;  
        float standard_4_x;  
        float standard_4_y;  
        float standard_5_x;  
        float standard_5_y; 
    } ground_robot_position_t;
    typedef struct dart_info_t
    { 
        uint8_t dart_remaining_time; 
        uint16_t dart_info; 
    }dart_info_t;
    typedef struct my_sentry_info_t
    {
        uint32_t exchange_bullet_num : 11;
        uint32_t remote_exchange_bullet_num :4;
        uint32_t remote_exchange_blood_num :4;
        uint32_t remain_data : 13;
    } my_sentry_info_t;

    typedef struct referee_robot_pos_t
    {
        float x;
        float y;
        float angle;
    }referee_robot_pos_t;

    typedef struct referee_game_state_t {
        game_type_e game_type;
        game_progress_e game_progress;
        ground_robot_position_t ground_robot_position;
        event_data_t event_data;
        dart_info_t dart_info;
        referee_robot_pos_t referee_robot_pos;
        my_sentry_info_t my_sentry_info;
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

    msg.hero_x = data.ground_robot_position.hero_x;
    msg.hero_y = data.ground_robot_position.hero_y;
    msg.engineer_x = data.ground_robot_position.engineer_x;
    msg.engineer_y = data.ground_robot_position.engineer_y;
    msg.standard_3_x = data.ground_robot_position.standard_3_x;
    msg.standard_3_y = data.ground_robot_position.standard_3_y;
    msg.standard_4_x = data.ground_robot_position.standard_4_x;
    msg.standard_4_y = data.ground_robot_position.standard_4_y;
    msg.standard_5_x = data.ground_robot_position.standard_5_x;
    msg.standard_5_y = data.ground_robot_position.standard_5_y;

    msg.supply_station_front_m = data.event_data.Supply_Station_front_M;
    msg.supply_station_in_m = data.event_data.Supply_Station_in_M;
    msg.supply_zone = data.event_data.Supply_zone;
    msg.power_rune_spot_status = data.event_data.power_rune_spot_status;
    msg.small_power_rune_status = data.event_data.small_power_rune_status;
    msg.big_power_rune_status = data.event_data.big_power_rune_status;
    msg.circular_highland_m = data.event_data.Circular_highland_M;
    msg.trapezium_highland_3_m= data.event_data.trapezium_highland_3_M;
    msg.trapezium_highland_4_m = data.event_data.trapezium_highland_4_M;
    msg.virtual_shield = data.event_data.Virtual_Shield;
    msg.ennmy_dart_info = data.event_data.ennmy_dart_info;
    msg.rfid_core_rmul = data.event_data.rfid_core_RMUL;

    msg.dart_remaining_time = data.dart_info.dart_remaining_time;
    msg.dart_info = data.dart_info.dart_info;

    msg.x = data.referee_robot_pos.x;
    msg.y = data.referee_robot_pos.y;
    msg.angle = data.referee_robot_pos.angle;
    msg.exchange_bullet_num = data.my_sentry_info.exchange_bullet_num ;
    msg.remote_exchange_blood_num = data.my_sentry_info.remote_exchange_blood_num;
    msg.remote_exchange_bullet_num = data.my_sentry_info.remote_exchange_bullet_num;
}
}