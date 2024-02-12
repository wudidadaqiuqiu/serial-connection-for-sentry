#include <cstddef>
#include <iostream>
#include <typeinfo>
#include <utility>

#include "easy_robot_commands/msg/robot_modules_mode.hpp"
#include "easy_robot_commands/shared_member/base_caller_template.hpp"
#include "easy_robot_commands/shared_member/pkg_info.hpp"
#include "easy_robot_commands/shared_member/trigger_operation.hpp"
#include "easy_robot_commands/stream.hpp"
#include "protocol/serialized_protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_connection/connector.hpp"

using module_msg = easy_robot_commands::msg::RobotModulesMode;
using chassis_ve_msg = easy_robot_commands::msg::RobotChassisVelocity;
using EasyRobotCommands::CRC16Config;
using EasyRobotCommands::ea_base_caller;
using EasyRobotCommands::protocol_type_e;
using EasyRobotCommands::ProtocolConfig;
using EasyRobotCommands::Stream;
using EasyRobotCommands::StructDataT;
using EasyRobotCommands::trigger_operation;
using SerialConection::Connector;

// using EasyRobotCommands::AllMSGPackT::CallerTuple;

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

template <typename TupleMSG>
struct SubsTupleT {
    static constexpr std::size_t MSG_NUN = std::tuple_size<TupleMSG>::value;

    template <std::size_t... Index>
    struct seq_geT {
        using seq = std::index_sequence<Index...>;
    };

    template <typename... MSGs>
    struct SubsTuple {
        using type = std::tuple<typename rclcpp::Subscription<MSGs>::SharedPtr...>;
    };

    template <std::size_t... Indices>
    static constexpr auto makeSubsTuple(std::index_sequence<Indices...>) ->
        typename SubsTuple<std::tuple_element_t<Indices, TupleMSG>...>::type {
        return SubsTuple<std::tuple_element_t<Indices, TupleMSG>...>::type();
    }

    using Type = decltype(makeSubsTuple(std::make_index_sequence<MSG_NUN>{}));
};

class ConnectorNode : public rclcpp::Node {
    using self_t = ConnectorNode;

   public:
    ConnectorNode()
        : Node("ConnectorNode"),
          tu(),
          //   robot_mode(),
          //   chassis_ve(),
          con("usb:v0483p5740d0200dc02dsc02dp00ic02isc02ip01in00", "/home/xy/code/ros2_ws/src/serial_connection/src/uart_fd.bash") {
        subscription_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor->add_callback_group(subscription_callback_group, this->get_node_base_interface());

        con.Connect();
    }

    template <typename MSG>
    void subscribe_easy_robot_command(rclcpp::Subscription<MSG>::SharedPtr& sub, ea_base_caller<MSG>& b) {
        auto lambda = [&b](typename MSG::SharedPtr msg) {
            b.triggered_from(msg);
        };
        rclcpp::SubscriptionOptions options;
        options.callback_group = subscription_callback_group;
        sub = create_subscription<MSG>("easy_robot_commands/" +
                                           std::string(StructDataT<MSG>::topic_name),
                                       rclcpp::SensorDataQoS(), lambda, options);
        con << (stream << b);
    }

    void subscribe_commands() {
        folded_subscribe(std::make_index_sequence<SubsTupleT<
                             EasyRobotCommands::AllMSGPackT::MSGTuple>::MSG_NUN>());
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

   private:
    // rclcpp::Subscription<module_msg>::SharedPtr sub;
    // rclcpp::Subscription<chassis_ve_msg>::SharedPtr sub2;
    // SubsTuple<std::tuple_element_t<0, EasyRobotCommands::AllMSGPackT::MSGTuple>>::type subtu;
    SubsTupleT<EasyRobotCommands::AllMSGPackT::MSGTuple>::Type subtu;

    EasyRobotCommands::AllMSGPackT::CallerTuple tu;

    // ea_base_caller<chassis_ve_msg> chassis_ve;
    Stream<20, ProtocolConfig<CRC16Config<0xFFFF, 0x1021>, protocol_type_e::protocol0>> stream;
    Connector con;

    rclcpp::CallbackGroup::SharedPtr subscription_callback_group;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConnectorNode>();
    node->subscribe_commands();
    node->executor_add_and_apin();
    rclcpp::shutdown();
    return 0;
}