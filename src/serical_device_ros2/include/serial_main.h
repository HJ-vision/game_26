#ifndef ROBOMASTER_ROBOT_H
#define ROBOMASTER_ROBOT_H//头文件保护

#include <iostream>
#include <thread>//多线程任务并行
#include <vector>//动态数组
#include "serial_device.h"
#include "protocol_new.hpp"//注意：用的是新协议，不是队里旧的 protocol.h
#include "crc.h"
#include <memory>//智能指针与内存管理

class SerialMain {
public:
	SerialMain(std::string device_path = "/dev/robomaster");//串口设备的默认路径，如果传入新路径则覆盖

	~SerialMain() = default;//析构函数，默认销毁

	// 发送数据。vdata 的 8 个元素必须按这个顺序来：
	// [0]yaw [1]yaw_vel [2]yaw_acc [3]pitch [4]pitch_vel [5]pitch_acc [6]target_lock [7]fire_command
	void SenderMain(const std::vector<double> &vdata);

	bool CommInit();//初始化串口

	bool ReceiverMain();// 读取数据（找帧头 + 双重 CRC 校验 + 拆帧）

	uint16_t ReceiveDataSolve(uint8_t *frame);//按 protocol_new.hpp 的协议格式拆解正文

	uint16_t SenderPackSolve(uint8_t *data, uint16_t data_length,
							 uint16_t cmd_id, uint8_t *send_buf);//打包发送帧

	io::VisionData vision_msg_;//下位机发来的云台姿态等数据存在这里

private:

	//! Device Information and Buffer Allocation
	std::string device_path_;//存储构造函数传入的串口设备路径
	std::shared_ptr<SerialDevice> device_ptr_;//智能指针，SerialDevice 是底层串口操作类
	std::unique_ptr<uint8_t[]> recv_buff_;//接收缓冲区，收到的原始字节存这里
	std::unique_ptr<uint8_t[]> send_buff_;//发送缓冲区，要发的帧拼好后存这里
	const unsigned int BUFF_LENGTH = 512;//串口每帧不会超过这个大小
	int pending_ = 0;//跨多次读取的字节累加计数，用于把被串口分两次到达的同一帧拼起来

	//! Frame Information
	io::FrameHeader frame_receive_header_;//记录收到的帧头信息
	io::FrameHeader frame_send_header_;//用来拼发送的帧头

	//! Protocol data
	io::GimbalCtrlData gimbal_ctrl;//待发送的云台控制数据，字段和 protocol_hj 的 GimbalCtrlData 一一对应（31 字节）

	// 新协议整帧结构（protocol_new.hpp）：
	// [SOF(1)][data_length(2)][seq(1)][CRC8(1)] 帧头 5 字节
	// [cmd_id(2)]                              2 字节
	// [正文 data_length 字节]
	// [CRC16(2)]                               覆盖 帧头+cmd_id+正文
	// [帧尾 0x0D 0x0A(2)]                       MsgEndInfo
	// 所以整帧总长 = data_length + 11
	static constexpr uint16_t FRAME_HEAD_LEN = 5;   // sizeof(io::FrameHeader)
	static constexpr uint16_t FRAME_CMD_LEN = 2;
	static constexpr uint16_t FRAME_CRC16_LEN = 2;
	static constexpr uint16_t FRAME_TAIL_LEN = 2;   // MsgEndInfo
	static constexpr uint16_t FRAME_OVERHEAD = 11;  // 5 + 2 + 2 + 2
};

#endif // ROBOMASTER_ROBOT_H
