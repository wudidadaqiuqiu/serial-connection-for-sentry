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