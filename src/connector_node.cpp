#include <cstddef>
#include <iostream>
#include <typeinfo>
#include <utility>
#include <chrono>

#include "easy_robot_commands/msg/robot_modules_mode.hpp"
#include "easy_robot_commands/msg/robot_shoot_info.hpp"
#include "easy_robot_commands/shared_member/base_caller_template.hpp"
#include "easy_robot_commands/shared_member/pkg_info.hpp"
#include "easy_robot_commands/shared_member/trigger_operation.hpp"
#include "easy_robot_commands/stream.hpp"
#include "protocol/serialized_protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_connection/connector.hpp"
#include "struct_def/struct_def.hpp"
#include "struct_def/transmit_info.hpp"
#include "struct_def/receive_info.hpp"
#include "struct_def/shoot_info.hpp"
#include "frame_rate.hpp"

#include "struct_def/geometry_msg_specified.hpp"
#include "struct_def/referee_data_for_decision.hpp"
#include "struct_def/offset_data.hpp"
#include "struct_def/game_state.hpp"

using namespace std::chrono_literals;
namespace msgea = easy_robot_commands::msg;
// using module_msg = easy_robot_commands::msg::RobotModulesMode;
// using chassis_ve_msg = easy_robot_commands::msg::RobotChassisVelocity;
// using sh_info_msg = easy_robot_commands::msg::RobotShootInfo;
using EasyRobotCommands::CRC16Config;
using EasyRobotCommands::ea_base_caller;
using EasyRobotCommands::protocol_pack_id;
using EasyRobotCommands::protocol_size_t;
using EasyRobotCommands::protocol_type_e;
using EasyRobotCommands::ProtocolConfig;
using EasyRobotCommands::Stream;
using EasyRobotCommands::StructDataT;
using EasyRobotCommands::trigger_operation;
using EasyRobotCommands::Unpacker;
using EasyRobotCommands::MSGPack;
using SerialConection::Connector;
using SerialConection::framerate_t;

using StructDef::shoot_info_t;
using StructDef::referee_data_for_decision;
using StructDef::robot_offset_data;
using StructDef::referee_game_state;

using TransmiteInfo::SubsTupleT;
using ReceiveInfo::PubsT;
using SubsMSG = MSGPack<geometry_msgs::msg::WrenchStamped>;

// 递归模板展开，对每个元素调用默认构造函数
template <typename Args>
auto make_tuple_with_default_constructors() {
    return typename Args::type();
}

void printArray(const uint8_t* arr, size_t length) {
    std::cout << std::hex;
    for (size_t i = 0; i < length; ++i) {
        std::cout << "0x" << static_cast<int>(arr[i]) << " ";
    }

    std::cout << std::dec;
    if (length != 0 && arr[length - 1] == 0x7e)
        std::cout << std::endl;
}

class PrintConsumer {
   public:
    static void blocked_consume(const uint8_t* data, size_t len) {
        printArray(data, len);
    }
};

class ConnectorNode : public rclcpp::Node {
    using self_t = ConnectorNode;

   public:
    ConnectorNode()
        : Node("ConnectorNode"),
          tu(),
        //   rate1("recv"),
          //   robot_mode(),
          //   chassis_ve(),
          con("usb:v0483p5740d0200dc02dsc02dp00ic02isc02ip01in00", "/home/ubuntu/sentry_ws/src/serial-connection-for-sentry/src/uart_fd.bash", printArray) {
        subscription_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor->add_callback_group(subscription_callback_group, this->get_node_base_interface());

        reconnect_timer = this->create_wall_timer(
        1000ms, std::bind(&ConnectorNode::check_connection, this));
        // pub = this->create_publisher<sh_info_msg>("easy_robot_commands/robot_shoot_info", 10);

        // auto l1 = [this](protocol_pack_id id, const uint8_t* data, protocol_size_t data_len) {
        //     (void)id;
        //     // std::cout << "update len is " << data_len << std::endl;
        //     // printArray(data, data_len);
        //     StructDef::shoot_info_t::shoot_info_to_pc_t* s = (StructDef::shoot_info_t::shoot_info_to_pc_t*) data;
        //     static_assert(sizeof(StructDef::shoot_info_t::shoot_info_to_pc_t) == 15);
        //     (sizeof(StructDef::shoot_info_t::shoot_info_to_pc_t));
        //     this->msgpub.rpy[0] = s->euler[0];
        //     this->msgpub.rpy[1] = s->euler[1];
        //     this->msgpub.rpy[2] = s->euler[2];
        //     this->msgpub.mode = static_cast<uint8_t>(s->auto_mode_flag);
        //     this->msgpub.robot_id = s->robot_id;
        //     this->msgpub.x = s->bullet_speed_0_to_127 | s->is_shoot_data_updated;
        //     this->pub->publish(this->msgpub);
        //     this->rate1.update();
        //     // std::cout << "rate: " << rate1.fps << std::endl;
        // };
        // auto ll1 = [](protocol_pack_id id) -> bool {
        //     // std::cout << "unpack id is " << id <<std::endl;
        //     return (id == 0x03);
        // };
        std::map<protocol_pack_id, std::function<void(protocol_pack_id, const uint8_t*, protocol_size_t)>> update_func_map;
        std::map<protocol_pack_id, std::function<bool(protocol_pack_id)>> check_id_func_map;
        // check_id_func_map[0x03] = ll1;
        // update_func_map[0x03] = l1;
        // shoot_info_pubt.init(*this);
        // shoot_info_pubt.add_to_maps(check_id_func_map, update_func_map);

        referee_data_pub.init(*this);
        referee_data_pub.add_to_maps(check_id_func_map, update_func_map);
        
        robot_offset_data_pub.init(*this);
        robot_offset_data_pub.add_to_maps(check_id_func_map, update_func_map);

        referee_game_state_pub.init(*this);
        referee_game_state_pub.add_to_maps(check_id_func_map, update_func_map);

        unpacker.change_map(update_func_map, check_id_func_map);

        auto c1 = [this](const uint8_t* data, size_t len) {
            unpacker.unpack(data, len);
        };
        con.change_recv_callbaclk(c1);
        con.Connect();
    }

    template <typename MSG>
    void subscribe_easy_robot_command(rclcpp::Subscription<MSG>::SharedPtr& sub, ea_base_caller<MSG>& b) {
        auto lambda = [&b](typename MSG::SharedPtr msg) {
            b.triggered_from(msg);
            // std::cout << "subs ..." << std::endl;
        };
        rclcpp::SubscriptionOptions options;
        options.callback_group = subscription_callback_group;
        sub = create_subscription<MSG>("/" +
                                           std::string(StructDataT<MSG>::topic_name),
                                       rclcpp::SensorDataQoS(), lambda, options);
        con << (stream << b);
    }

    void subscribe_commands() {
        folded_subscribe(std::make_index_sequence<SubsTupleT<
                             SubsMSG::MSGTuple>::MSG_NUN>());
    }

    // template <std::size_t Index = 0, typename... Types>
    // void tuple_subscribe_commands_helper(std::tuple<Types...>& t) {
    //     if constexpr (Index < sizeof...(Types)) {
    //         // 调用 subscribe_easy_robot_command，并传递元组中的当前元素
    //         subscribe_easy_robot_command(std::get<Index>(t));
    //         // 递归展开下一个元素
    //         tuple_subscribe_commands_helper<Index + 1>(t);
    //     }
    // }

    // // 主函数：调用辅助函数
    // template <typename... Types>
    // void tuple_subscribe_commands(std::tuple<Types...>& t) {
    //     tuple_subscribe_commands_helper(t);
    // }

    template <size_t... Indices>
    void folded_subscribe(std::index_sequence<Indices...>) {
        (subscribe_easy_robot_command(std::get<Indices>(subtu), std::get<Indices>(tu)), ...);
    }

    void executor_add_and_apin() {
        executor->add_node(shared_from_this());
        executor->spin();
    };

    void check_connection() {
        if (!con.is_working()) {
            con.ReConnect();
        }
    }

   private:
    // rclcpp::Subscription<module_msg>::SharedPtr sub;
    // rclcpp::Subscription<chassis_ve_msg>::SharedPtr sub2;
    // SubsTuple<std::tuple_element_t<0, EasyRobotCommands::AllMSGPackT::MSGTuple>>::type subtu;
    // SubsTupleT<EasyRobotCommands::AllMSGPackT::MSGTuple>::Type subtu;
    SubsTupleT<SubsMSG::MSGTuple>::Type subtu;

    SubsMSG::CallerTuple tu;
    // EasyRobotCommands::AllMSGPackT::CallerTuple tu;

    // rclcpp::Publisher<sh_info_msg>::SharedPtr pub;
    // sh_info_msg msgpub;
    // framerate_t rate1;

    // PubsT<sh_info_msg, shoot_info_t> shoot_info_pubt;
    PubsT<robot_msgs::msg::RefereeData, referee_data_for_decision> referee_data_pub;

    PubsT<robot_msgs::msg::GimbalData, robot_offset_data> robot_offset_data_pub;

    PubsT<robot_msgs::msg::RefereeGameState, referee_game_state> referee_game_state_pub;

    // ea_base_caller<chassis_ve_msg> chassis_ve;
    Stream<20, ProtocolConfig<CRC16Config<0xFFFF, 0x1021>, protocol_type_e::protocol0>> stream;
    Unpacker<ProtocolConfig<CRC16Config<0xFFFF, 0x1021>, protocol_type_e::protocol0>> unpacker;
    Connector con;

    rclcpp::CallbackGroup::SharedPtr subscription_callback_group;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
    rclcpp::TimerBase::SharedPtr reconnect_timer;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConnectorNode>();
    node->subscribe_commands();
    node->executor_add_and_apin();
    rclcpp::shutdown();
    return 0;
}