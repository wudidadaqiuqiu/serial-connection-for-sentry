#pragma once

#include "rclcpp/rclcpp.hpp"
#include "protocol/serialized_protocol.hpp"
#include "struct_def/struct_def.hpp"
#include "frame_rate.hpp"

namespace ReceiveInfo {
using EasyRobotCommands::protocol_pack_id;
using EasyRobotCommands::protocol_size_t;
using EasyRobotCommands::whole_pkg_check_func;
using EasyRobotCommands::update_pkg_func;

template <typename MSGT, typename PubStructT> requires StructDef::ValidReceiveStruct<MSGT, PubStructT>
struct PubsT;

template <typename MSGT, typename PubStructT> requires StructDef::ValidReceiveStruct<MSGT, PubStructT>
void add_whole_pkg_check_to_map(std::map<protocol_pack_id, whole_pkg_check_func>& map, 
                                        PubsT<MSGT, PubStructT>& pp);

template <typename MSGT, typename PubStructT> requires StructDef::ValidReceiveStruct<MSGT, PubStructT>
void add_update_pkg_to_map(std::map<protocol_pack_id, update_pkg_func>& map, 
                                        PubsT<MSGT, PubStructT>& pp);

template <typename MSGT, typename PubStructT> requires StructDef::ValidReceiveStruct<MSGT, PubStructT>
struct PubsT {
    rclcpp::Publisher<MSGT>::SharedPtr pub;
    MSGT msg;
    PubStructT s;
    SerialConection::framerate_t framerate;
    bool has_init;
    PubsT() : framerate(std::string(PubStructT::sub_topic) + "_recv"), has_init(false) {}
    void init(rclcpp::Node& node) {
        pub = node.create_publisher<MSGT>("easy_robot_commands/" + std::string(PubStructT::sub_topic), 10);
        has_init = true;
    }
    void add_to_maps(std::map<protocol_pack_id, whole_pkg_check_func>& whole_check_map, 
                        std::map<protocol_pack_id, update_pkg_func>& update_map) {
        if (has_init == false) {
            std::cerr << "has not init" << std::endl;
            throw;
        }
        add_whole_pkg_check_to_map(whole_check_map, *this);
        add_update_pkg_to_map(update_map, *this);
    }
};

template <typename MSGT, typename PubStructT> requires StructDef::ValidReceiveStruct<MSGT, PubStructT>
void add_whole_pkg_check_to_map(std::map<protocol_pack_id, whole_pkg_check_func>& map, 
                                        PubsT<MSGT, PubStructT>& pp) {
    auto ll1 = [](protocol_pack_id id) -> bool {
        (void)id;
        // std::cout << "unpack id is " << id <<std::endl;
        return true;
    };
    map[pp.s.ID] = ll1;
}

template <typename MSGT, typename PubStructT> requires StructDef::ValidReceiveStruct<MSGT, PubStructT>
void add_update_pkg_to_map(std::map<protocol_pack_id, update_pkg_func>& map, 
                                        PubsT<MSGT, PubStructT>& pp) {
    auto l1 = [&pp](protocol_pack_id id, const uint8_t* data, protocol_size_t data_len) {
        (void)id;
        // std::cout << "update pkg" << std::endl;
        // if (data_len != sizeof(pp.s.data)) {
        //     // std::cout << "throw error" << std::endl;
        //     std::cerr << "len not equal!" << data_len << " != " << sizeof(pp.s.data) << std::endl;
        //     throw;
        // }
        // PubStructT* s = (PubStructT*) data;
        memcpy(&pp.s.data, data, data_len);
        // std::cout << "memcpy\n";
        pp.s.transfer_to(pp.msg);
        pp.pub->publish(pp.msg);
        pp.framerate.update();
        // std::cout << "update complete" << std::endl;
    };
    map[pp.s.ID] = l1;
}


}