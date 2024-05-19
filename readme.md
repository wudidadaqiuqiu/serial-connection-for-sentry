# sentry connection doc
- 23.3.13
## launch
```shell
ubuntu@user:~$ source bridge.sh
```
## node info
```shell
ubuntu@user:~/sentry_ws$ ros2 node info /ConnectorNode 
/ConnectorNode
  Subscribers:
    /cmd_vel: geometry_msgs/msg/WrenchStamped
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /easy_robot_commands/offset_data: robot_msgs/msg/GimbalData
    /easy_robot_commands/referee_data_for_decision: robot_msgs/msg/RefereeData
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /ConnectorNode/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /ConnectorNode/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /ConnectorNode/get_parameters: rcl_interfaces/srv/GetParameters
    /ConnectorNode/list_parameters: rcl_interfaces/srv/ListParameters
    /ConnectorNode/set_parameters: rcl_interfaces/srv/SetParameters
    /ConnectorNode/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```
## 消息处理
### chip -> ros2
- /easy_robot_commands/referee_data_for_decision: robot_msgs/msg/RefereeData
    - 结构体到msg的赋值逻辑详见 src/serial-connection-for-sentry/include/struct_def/referee_data_for_decision.hpp 中的transfer_to 函数
- /easy_robot_commands/offset_data: robot_msgs/msg/GimbalData
    - 结构体到msg的赋值逻辑详见 src/serial-connection-for-sentry/include/struct_def/offset_data.hpp 中的transfer_to 函数
### ros2 -> chip
- /cmd_vel: geometry_msgs/msg/WrenchStamped
    - src/serial-connection-for-sentry/include/struct_def/geometry_msg_specified.hpp 中声明了消息转换到结构体的函数

# serial connection 使用文档

## protocol
使用模板参数来选择不同的协议
```c++
Stream<20, ProtocolConfig<CRC16Config<0xFFFF, 0x1021>, protocol_type_e::protocol0>> stream;
Unpacker<ProtocolConfig<CRC16Config<0xFFFF, 0x1021>, protocol_type_e::protocol0>> unpacker;
```
其中`Stream`为存储发送字节的对象，`unpacker`是接收用对象。
- protocol_type_e::protocol0
    - crc16 = crc_calc(id + data)
    - escaped_data = escaped(data)
    - wholeframe = 0x7d + id(1 byte) + escaped_data(n bytes) + crc16(2 bytes) + 0x7e
    - 配套的嵌入式端驱动在[framework HAL lib](https://gl.ngxy.team:32072/embedded_group/projects/all_in_one_framework/hal)
    - 支持半包、粘包发送
## ros2端发送
- 在`connector_node.cpp`中使用了`std::tuple`作为若干个subscription和若干个ea_base_caller的容器。
- 若要修改添加要发送的结构体，需在`connector_node.cpp`中修改`using SubsMSG = MSGPack<msg1, msg2, /*...*/>;`中的模板参数，模板参数为ros2中定义的msg，并且在某个头文件中实现`EasyRobotCommands::StructDataT`的模板特化。模板特化的例子在`src/easy_robot_commands/include/easy_robot_commands/base_caller_specified`文件夹中。
- subscribe topic为`easy_robot_commands/your_subtopic_name_defined_in_StructDataT_specified`
## ros2端接收
- 采用定义好的模板对象`template <typename MSGT, typename PubStructT> requiresStructDef::ValidReceiveStruct<MSGT,PubStructT> struct PubsT;`存储ros2中的msg，publisher，并集成一些便捷的功能--计算接收帧率。`Connector`中使用接收线程接收数据，向上传递给`unpacker`，进而解包后publish。
- 要接收一个特定的包，需要在某个头文件中声明好一个结构体，例子在`src/serial_connection/include/struct_def/shoot_info.hpp`中，结构体的要求在requires concepts中已说明。然后，需要在`ConnectorNode`中增加相应的成员，调用相应的初始化函数，如:
```c++
// 成员声明
PubsT<sh_info_msg, shoot_info_t> shoot_info_pubt;
```
```c++
/* 局部变量map 定义...*/

// 初始化示例
shoot_info_pubt.init(*this);
shoot_info_pubt.add_to_maps(check_id_func_map, update_func_map);
//其他PubsT的初始化...

/*配置unpacker 回调map...*/
/*链接unpacker 与 connector...*/
```
- publish topic为`easy_robot_commands/your_subtopic_name_defined_in_struct_you_defined`

## TODO
- PubsT中MSGT可以改成参数包，以便更灵活地操作接收struct，可以实现一个结构体数据处理成多个msg。

## 改进与扩展可能
- 泛化connector
- protocol类型增加



## 测试
```shell
ros2 topic pub /cmd_vel geometry_msgs/msg/WrenchStamped "{
    header: {
        stamp: {sec: 0, nanosec: 0},
        frame_id: 'base_link'
    },
    wrench: {
        force: {x: 1.0, y: 0.0, z: 0.0},
        torque: {x: 0.0, y: 1.0, z: 0.0}
    }
}"


ros2 topic pub /walk_cmd robot_msgs/msg/WalkCmd "{
  opt: 10,
  radium: 1.5,
  velocity: 300.0,
  pos: {x: 1.0, y: 2.0, z: 3.0},
  cap_mode: 1
}"


ros2 topic pub /easy_robot_commands/decision_points robot_msgs/msg/DecisionPoints "{
  intention: 1,
  start: {x: 0.0, y: 0.0, z: 0.0},
  point1: {x: 1.0, y: 1.0, z: 0.0},
  point2: {x: 2.0, y: 2.0, z: 0.0}
}"
```
