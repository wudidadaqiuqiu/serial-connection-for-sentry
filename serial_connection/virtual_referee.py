import rclpy
from rclpy.node import Node

from robot_msgs.msg import RefereeData, RobotBattleState
from robot_msgs.msg import RefereeGameState

import yaml

def modify_referee_data(a: dict, b: RefereeData):
    if ('allow_bullet' in a.keys() and b.allow_bullet != a["allow_bullet"]):
        b.allow_bullet = a['allow_bullet']
    if ('robot_id' in a.keys() and b.robot_id != a["robot_id"]):
        b.robot_id = a['robot_id']
    # if ('robot_id' in a.keys() and b.robot_id != a["robot_id"]):
    #     b.robot_id = a['robot_id']
    for i in range(16):
        b.data[i].id = a['id'][i]
        b.data[i].blood = a['blood'][i]
    pass

class MyNode(Node):
    def __init__(self):
        super().__init__('virtual_referee')
        self.referee_info = RefereeData()
        for i in range(16):
            self.referee_info.data.append(RobotBattleState())

        self.f_topic = "/easy_robot_commands"
        # 读取 YAML 文件
        with open('/home/ubuntu/sentry_ws/src/serial-connection-for-sentry/serial_connection/referee_config.yaml', 'r') as file:
            config = yaml.safe_load(file)
        for c in config:
            self.declare_parameter(c, config[c])
        # self.declare_parameter('my_param', 'default_value')
        print(config)
        self.game_time = config['game_time_start']
        self.config_game_time = self.game_time

        modify_referee_data(config, self.referee_info)
        self.timer = self.create_timer(1, self.timer_callback)
        self.publisher_ = self.create_publisher(RefereeData, self.f_topic+"/referee_data_for_decision", 10)
        self.pub_timer = self.create_timer(0.1, self.timer_callback2)

        # self.timer = se
        # my_param_value = self.get_parameter('robot_id').value
        # self.get_logger().info('robot_id value: %s' % my_param_value)

        # my_param_value = self.get_parameter('game_time_start').value
        # self.get_logger().info('game_time_start value: %s' % my_param_value)
    def timer_callback(self):
        config: dict = {}
        with open('/home/ubuntu/sentry_ws/src/serial-connection-for-sentry/serial_connection/referee_config.yaml', 'r') as file:
            config = yaml.safe_load(file)
        if not config:
            print("read yaml fail")
            return
        
        if self.config_game_time != config['game_time_start']:
            self.config_game_time = config['game_time_start']
            self.game_time = config['game_time_start']
            self.referee_info.game_time = self.game_time + 1

        if (self.referee_info.game_time > 0):
            self.referee_info.game_time -= 1
        else:
            self.referee_info.game_time = self.game_time
        
        # if ('allow_bullet' in config.keys() and self.get_parameter('allow_bullet') != config["allow_bullet"]):
        modify_referee_data(config, self.referee_info)

        # self.publisher_.publish(self.referee_info)
        # print("allow_bullet: ", self.referee_info.allow_bullet)

    def timer_callback2(self):
        self.publisher_.publish(self.referee_info)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nctrl C end")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

