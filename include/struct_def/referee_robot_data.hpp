#pragma once
#include <cstdint>
// #include <string>
#include <cstring>
#include "robot_msgs/msg/referee_robot_data.hpp"

namespace StructDef {

struct referee_robot_data {
    typedef  struct buff_t{  
        uint8_t recovery_buff; 
        uint8_t cooling_buff; 
        uint8_t defence_buff; 
        uint8_t vulnerability_buff; 
        uint16_t attack_buff; 
    }buff_t;
    typedef struct rfid_status_t
    { 
        uint8_t rfid_base : 1;
        uint8_t rfid_Circular_highland_M : 1;
        uint8_t rfid_Circular_highland_E : 1;
        uint8_t rfid_highland_3_M : 1;
        uint8_t rfid_highland_3_E : 1;
        uint8_t rfid_highland_4_M : 1;
        uint8_t rfid_highland_4_E : 1;
        uint8_t rfid_Energy_mechanism : 1;
        uint8_t rfid_fly_front_M : 1;
        uint8_t rfid_fly_end_M : 1;
        uint8_t rfid_fly_front_E : 1;
        uint8_t rfid_fly_end_E : 1;
        uint8_t rfid_outpost : 1;
        uint8_t rfid_bloodadding : 1;
        uint8_t rfid_patrol_M : 1;
        uint8_t rfid_patrol_E : 1;
        uint8_t rfid_Big_Resource_Island_M : 1;
        uint8_t rfid_Big_Resource_Island_E : 1;
        uint8_t rfid_exchange_zone : 1;
        uint8_t rfid_core : 1;
        uint16_t rfid_rest : 12;
    }rfid_status_t; 

    #pragma pack(1)
    typedef struct referee_robot_data_t {
        buff_t robot_buff;
        rfid_status_t robot_rfid;
    } referee_robot_data_t;
    #pragma pack()
    referee_robot_data_t data;
    static constexpr uint8_t ID = 0x17;
    static constexpr std::string_view sub_topic = "referee_robot_data";
    template <typename T>
    void transfer_to(T& msg);
};

template <>
void referee_robot_data::transfer_to<robot_msgs::msg::RefereeRobotData>(robot_msgs::msg::RefereeRobotData& msg) {
    msg.attack_buff = data.robot_buff.attack_buff;
    msg.cooling_buff = data.robot_buff.cooling_buff;
    msg.defence_buff = data.robot_buff.defence_buff;
    msg.recovery_buff = data.robot_buff.recovery_buff; 
    msg.vulnerability_buff= data.robot_buff.vulnerability_buff;

    msg.rfid_base = data.robot_rfid.rfid_base;
    msg.rfid_big_resource_island_e = data.robot_rfid.rfid_Big_Resource_Island_E;
    msg.rfid_big_resource_island_m = data.robot_rfid.rfid_Big_Resource_Island_M;
    msg.rfid_bloodadding = data.robot_rfid.rfid_bloodadding;
    msg.rfid_circular_highland_e = data.robot_rfid.rfid_Circular_highland_E;
    msg.rfid_circular_highland_m = data.robot_rfid.rfid_Circular_highland_M;
    msg.rfid_core = data.robot_rfid.rfid_core;
    msg.rfid_energy_mechanism = data.robot_rfid.rfid_Energy_mechanism;
    msg.rfid_exchange_zone = data.robot_rfid.rfid_exchange_zone;
    msg.rfid_fly_end_e = data.robot_rfid.rfid_fly_end_E;
    msg.rfid_fly_end_m = data.robot_rfid.rfid_fly_end_M;
    msg.rfid_fly_front_e = data.robot_rfid.rfid_fly_front_E;
    msg.rfid_fly_front_m = data.robot_rfid.rfid_fly_front_M;
    msg.rfid_highland_3_e = data.robot_rfid.rfid_highland_3_E;
    msg.rfid_highland_3_m = data.robot_rfid.rfid_highland_3_M;
    msg.rfid_highland_4_e = data.robot_rfid.rfid_highland_4_E;
    msg.rfid_highland_4_m = data.robot_rfid.rfid_highland_4_M;
    msg.rfid_outpost = data.robot_rfid.rfid_outpost;
    msg.rfid_patrol_e = data.robot_rfid.rfid_patrol_E;
    msg.rfid_patrol_m = data.robot_rfid.rfid_patrol_M;
    msg.rfid_rest = data.robot_rfid.rfid_rest;
}
}