#include <cstddef>
#include <iostream>
#include <typeinfo>

#include "easy_robot_commands/base_caller_specified/robot_modules_mode.hpp"
#include "easy_robot_commands/msg/robot_modules_mode.hpp"
#include "easy_robot_commands/shared_member/base_caller_template.hpp"
#include "easy_robot_commands/shared_member/trigger_operation.hpp"
#include "easy_robot_commands/stream.hpp"
#include "protocol/serialized_protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_connection/connector.hpp"

using module_msg = easy_robot_commands::msg::RobotModulesMode;

using EasyRobotCommands::CRC16Config;
using EasyRobotCommands::protocol_type_e;
using EasyRobotCommands::ProtocolConfig;
using EasyRobotCommands::StructDataT;
using EasyRobotCommands::trigger_operation;
using EasyRobotCommands::ea_base_caller;
using EasyRobotCommands::Stream;

using SerialConection::Connector;

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
          robot_mode(),
          con("usb:v0483p5740d0200dc02dsc02dp00ic02isc02ip01in00", "/home/xy/code/ros2_ws/src/serial_connection/src/uart_fd.bash") {
        subscription_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor->add_callback_group(subscription_callback_group, this->get_node_base_interface());

        con.Connect();
    }

    void subscribe_easy_robot_commands() {
        auto lambda1 = [this](module_msg::SharedPtr msg) {
            robot_mode.triggered_from(msg);
        };
        auto lambda2 = [this](ea_base_caller<module_msg>& caller) {
            // this->robot_modules_mode_triggered(caller);
            stream.triggered_from(caller);
        };
        auto lambda3 = [this](decltype(stream)& s) {
            static_assert(std::is_same<decltype(s),
                                       Stream<20, ProtocolConfig<CRC16Config<0xFFFF, 0x1021>, protocol_type_e::protocol0>>&>::value,
                          "Type mismatch!");
            // PrintConsumer c{};
            s >> con;
        };
        stream.register_trigger_operation(trigger_operation<decltype(stream)>(lambda3));
        robot_mode.register_trigger_operation(trigger_operation<ea_base_caller<module_msg>>(lambda2));
        rclcpp::SubscriptionOptions options;
        options.callback_group = subscription_callback_group;
        sub = create_subscription<module_msg>("easy_robot_commands/" +
                                                  std::string(StructDataT<module_msg>::topic_name),
                                              rclcpp::SensorDataQoS(), lambda1, options);
    }

    void executor_add_and_apin() {
        executor->add_node(shared_from_this());
        executor->spin();
    };

   private:
    rclcpp::Subscription<module_msg>::SharedPtr sub;
    ea_base_caller<module_msg> robot_mode;
    Stream<20, ProtocolConfig<CRC16Config<0xFFFF, 0x1021>, protocol_type_e::protocol0>> stream;

    Connector con;

    rclcpp::CallbackGroup::SharedPtr subscription_callback_group;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
    void robot_modules_mode_triggered(ea_base_caller<module_msg>& caller) {
        RCLCPP_INFO(this->get_logger(), "%d....", *(uint8_t*)&robot_mode.get_struct_data());
        RCLCPP_INFO(this->get_logger(), "%s: %d", std::string(caller.get_struct_data().MsgTypeName).c_str(), caller.get_structure_data().master_enable);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConnectorNode>();
    node->subscribe_easy_robot_commands();

    node->executor_add_and_apin();
    rclcpp::shutdown();
    return 0;
}