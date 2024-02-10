#include <signal.h>

#include <cstddef>

#include "container/multithread_stream.hpp"
#include "easy_robot_commands/base_caller_specified/robot_modules_mode.hpp"
#include "easy_robot_commands/msg/robot_modules_mode.hpp"
#include "easy_robot_commands/register_commands.hpp"
#include "easy_robot_commands/shared_member/base_caller_template.hpp"
#include "easy_robot_commands/shared_member/trigger_operation.hpp"
#include "easy_robot_commands/stream.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_connection/connector.hpp"

using module_msg = easy_robot_commands::msg::RobotModulesMode;
using EasyRobotCommands::ea_base_caller;
using EasyRobotCommands::multithread_stream;
using EasyRobotCommands::StructDataT;
using EasyRobotCommands::trigger_operation;

using SerialConection::Connector;
void printArray(const uint8_t* arr, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        // 使用setw设置每个元素的打印宽度
        std::cout << static_cast<char>(arr[i]);
    }
    std::cout << std::endl;
}

class TestNode : public rclcpp::Node {
   public:
    TestNode()
        : Node("TestNode"),
          robot_mode(),
          con("usb:v0483p5740d0200dc02dsc02dp00ic02isc02ip01in00", "/home/xy/code/ros2_ws/src/serial_connection/src/uart_fd.bash") {
        (void)executor_;
        // 创建两个不同的CallbackGroup，一个用于Subscription，另一个用于Timer
        subscription_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // 创建Timer并将其添加到Timer的CallbackGroup中
        // timer_ = this->create_wall_timer(std::chrono::seconds(1),
        //                                  std::bind(&TestNode::timer_callback, this), timer_callback_group_);

        // subss();
        // 启动Executor
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_callback_group(subscription_callback_group_, this->get_node_base_interface());
        // executor_->add_callback_group(timer_callback_group_, this->get_node_base_interface());
        // executor_->
        // create_subscription()
        // EasyRobotCommands::create_subscription_with_callback<easy_robot_commands::msg::RobotModulesMode>(shared_this, arr);
    }
    void subscribe_easy_robot_commands() {
        auto lll = [this](ea_base_caller<module_msg>& caller) {
            this->robot_modules_mode_triggered(caller);
        };
        robot_mode.register_trigger_operation(trigger_operation<ea_base_caller<module_msg>>(lll));
        auto lambda = [this](module_msg::SharedPtr msg) {
            robot_mode.triggered_from(msg);
        };
        rclcpp::SubscriptionOptions options;
        options.callback_group = subscription_callback_group_;
        sub = create_subscription<module_msg>("easy_robot_commands/" +
                                                  std::string(StructDataT<module_msg>::topic_name),
                                              rclcpp::SensorDataQoS(), lambda, options);
    }

    void subss() {
        auto lambda = [this](const module_msg::SharedPtr msg) {
            (void)msg;
            // 休眠1秒钟
            // rclcpp::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(this->get_logger(), "Subscription callback: ");

            stream.consume_with_best_effort(printArray, true);
        };
        rclcpp::SubscriptionOptions options;
        options.callback_group = subscription_callback_group_;

        sub = create_subscription<module_msg>("easy_robot_commands/" + std::string(StructDataT<module_msg>::topic_name), rclcpp::SensorDataQoS(),
                                              lambda, options);
    }

    void executor_add_and_apin() {
        executor_->add_node(shared_from_this());
        executor_->spin();
    };

   private:
    void robot_modules_mode_triggered(ea_base_caller<module_msg>& caller) {
        RCLCPP_INFO(this->get_logger(), "%d....", *(uint8_t*)&robot_mode.get_struct_data());
        RCLCPP_INFO(this->get_logger(), "%s: %d", std::string(caller.get_struct_data().MsgTypeName).c_str(), caller.get_structure_data().master_enable);
    }

    void timer_callback() {
        RCLCPP_INFO(get_logger(), "Timer callback triggered");
        multithread_stream<20>::stream_state_e s1 = stream.add_with_caution((const uint8_t*)"122333", 6);
        multithread_stream<20>::stream_state_e s2 = stream.add_with_caution((const uint8_t*)"644555", 6);

        std::cout << s1 << "    " << s2 << std::endl;
    };
    rclcpp::Subscription<module_msg>::SharedPtr sub;
    ea_base_caller<module_msg> robot_mode;
    multithread_stream<20> stream;

    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

    Connector con;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestNode>();
    node->subscribe_easy_robot_commands();

    node->executor_add_and_apin();
    rclcpp::shutdown();
    return 0;
}