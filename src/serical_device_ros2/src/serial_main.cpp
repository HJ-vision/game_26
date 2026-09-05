#include "serial_main.h"//包含自己的头文件
#include <cstring>//memcpy / memmove

SerialMain::SerialMain(std::string device_path) : device_path_(device_path)//构造函数不写返回值，初始化列表
{
	if (!(CommInit()))//成功不输出，失败输出，当if（）括号内为true时才输出
	{
		std::cout << "serial init error!!!!!!!!!!" << std::endl;//“初始化失败”
	}
}

void SerialMain::SenderMain(const std::vector<double> &vdata)
{
	// vdata 顺序和 RobotCtrl.msg 的字段一一对应：
	// [0]yaw [1]yaw_vel [2]yaw_acc [3]pitch [4]pitch_vel [5]pitch_acc [6]target_lock [7]fire_command
	if (vdata.size() < 8)
	{
		std::cout << "SenderMain: vdata size < 8, skip!" << std::endl;
		return;
	}
	// 按 protocol_hj 的 GimbalCtrlData（31 字节）填字段，顺序不能错：
	// yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc, main_yaw, fire_command, fire_mode, target_lock
	gimbal_ctrl.yaw = vdata[0];
	gimbal_ctrl.yaw_vel = vdata[1];
	gimbal_ctrl.yaw_acc = vdata[2];
	gimbal_ctrl.pitch = vdata[3];
	gimbal_ctrl.pitch_vel = vdata[4];
	gimbal_ctrl.pitch_acc = vdata[5];
	gimbal_ctrl.main_yaw = 0.0f;//哨兵底盘主 yaw，步兵填 0
	gimbal_ctrl.fire_command = static_cast<int8_t>(vdata[7]);//0 不发射 / 1 发射
	gimbal_ctrl.fire_mode = 1;//发射模式固定 1，电控默认值
	gimbal_ctrl.target_lock = static_cast<int8_t>(vdata[6]);//49 锁定 / 50 未锁定

	uint16_t send_length = SenderPackSolve((uint8_t *)&gimbal_ctrl, sizeof(io::GimbalCtrlData),
										   io::VISION_CTRL_CMD_ID, send_buff_.get());//正文 31 字节；视觉控制帧 cmd_id 0x0102；返回整帧总长度
	device_ptr_->Write(send_buff_.get(), send_length);//发出信息
}

bool SerialMain::CommInit()
{
	device_ptr_ = std::make_shared<SerialDevice>(device_path_, 115200); // 串口路径；波特率 115200，必须和电控一致

	if (!device_ptr_->Init())//真正打开和配置串口，失败返回false
	{
		return false;
	}

	recv_buff_ = std::make_unique<uint8_t[]>(BUFF_LENGTH);
	send_buff_ = std::make_unique<uint8_t[]>(BUFF_LENGTH);//初始化缓冲区，长度512字节

	memset(&frame_receive_header_, 0, sizeof(io::FrameHeader));
	memset(&frame_send_header_, 0, sizeof(io::FrameHeader));//memory set 内存设置，把帧头清0避免脏数据

	return true;
}

bool SerialMain::ReceiverMain()
{
	if (pending_ >= (int)BUFF_LENGTH)
	{
		pending_ = 0; // 缓冲区满了还没拼出合法帧，整个丢掉重来，防止越界
	}

	// 先把这次读到的原始字节放进临时缓冲，再追加到 recv_buff_ 末尾（跨多次读取拼接成完整帧）
	uint8_t temp[BUFF_LENGTH];
	int len = device_ptr_->Read(temp, BUFF_LENGTH - pending_);
	if (len <= 0)
	{
		return false;
	}
	memcpy(recv_buff_.get() + pending_, temp, len);
	pending_ += len;

	bool get = false;
	uint16_t i = 0;
	while (i + FRAME_HEAD_LEN <= (uint16_t)pending_)
	{
		if (recv_buff_[i] != io::HEADER_SOF)//当前字节不是帧头 0xA5
		{
			i++;//往后找
			continue;
		}

		// 拷出帧头（5 字节：sof + data_length + seq + crc8）
		memcpy(&frame_receive_header_, recv_buff_.get() + i, FRAME_HEAD_LEN);

		// 第一重校验：帧头 CRC8（覆盖帧头前 4 字节，校验码在第 5 字节）
		if (!Verify_CRC8_Check_Sum(recv_buff_.get() + i, FRAME_HEAD_LEN))
		{
			i++;//这个 0xA5 是正文里碰巧出现的，不是真帧头，往后找
			continue;
		}

		uint16_t data_length = frame_receive_header_.data_length;

		// 到 CRC16 为止需要的长度 = 帧头5 + cmd_id2 + 正文 + CRC16 2 = data_length + 9
		uint16_t frame_len_crc = data_length + FRAME_HEAD_LEN + FRAME_CMD_LEN + FRAME_CRC16_LEN;
		if ((uint16_t)pending_ - i < frame_len_crc)
		{
			break;//整帧还没到齐，留在缓冲区里等下次读取拼接
		}

		// 第二重校验：整帧 CRC16
		// Verify_CRC16_Check_Sum(buf, frame_len_crc) 的含义：
		//   对前 frame_len_crc - 2 = data_length + 7 字节（帧头+cmd_id+正文）算 CRC16，
		//   和 buf[data_length+7]、buf[data_length+8]（小端：低字节在前）比较
		uint8_t *frame = recv_buff_.get() + i;
		if (!Verify_CRC16_Check_Sum(frame, frame_len_crc))
		{
			std::cout << "CRC16 error! skip 1 byte" << std::endl;
			i++;//整帧 CRC 错，跳过这个 0xA5 继续找下一个帧头
			continue;
		}

		// CRC16 通过后看后面有没有帧尾 0x0D 0x0A（protocol_new.hpp 的 MsgEndInfo，25 赛季电控确实会发）
		uint16_t total_len = frame_len_crc;
		if ((uint16_t)pending_ - i >= frame_len_crc + FRAME_TAIL_LEN &&
			frame[frame_len_crc] == io::END1_SOF &&
			frame[frame_len_crc + 1] == io::END2_SOF)
		{
			total_len = frame_len_crc + FRAME_TAIL_LEN;//带帧尾的整帧长 = data_length + 11
		}

		ReceiveDataSolve(frame);//解出 vision_msg_
		get = true;
		i += total_len;//跳过整帧，继续找下一帧
	}

	// 把已经处理完的字节丢掉，剩下的未处理部分移到缓冲区开头，留待下次拼接
	if (i > 0)
	{
		memmove(recv_buff_.get(), recv_buff_.get() + i, pending_ - i);
		pending_ -= i;
	}
	return get;
}

uint16_t SerialMain::ReceiveDataSolve(uint8_t *frame)
{
	uint16_t index = 0;//游标，记录当前读到帧的哪个位置
	uint16_t cmd_id = 0;

	if (*frame != io::HEADER_SOF)
	{
		return 0;//不是合法帧头，终止函数
	}

	// CRC 双重校验已经在 ReceiverMain 里做过了，这里只负责拆包

	memcpy(&frame_receive_header_, frame, FRAME_HEAD_LEN);//拷贝帧头
	index += FRAME_HEAD_LEN;//游标跳到帧头之后

	memcpy(&cmd_id, frame + index, sizeof(uint16_t));//拷贝消息类型
	index += sizeof(uint16_t);//游标跳到消息类型之后

	switch (cmd_id)
	{
		case io::VISION_ID://下位机发来的视觉数据（IMU 姿态等），protocol_hj 里是 0x0104
		{
			// 防御：电控的正文长度必须和 io::VisionData（49 字节）对得上才拷贝，
			// 否则说明两端结构体版本不一致，拷了也是错数据
			if (frame_receive_header_.data_length >= sizeof(io::VisionData))
			{
				memcpy(&vision_msg_, frame + index, sizeof(io::VisionData));//正文拷贝进 vision_msg_
			}
			else
			{
				std::cout << "VISION frame data_length=" << frame_receive_header_.data_length
						  << " != " << sizeof(io::VisionData)
						  << " (电控和视觉的 VisionData 结构体不一致!)" << std::endl;
			}
			break;
		}
		default://其他消息类型，暂时不处理
			break;
	}

	index += frame_receive_header_.data_length + FRAME_CRC16_LEN + FRAME_TAIL_LEN;//游标移到帧尾之后
	return index;//返回整帧长度（含帧尾）
}

uint16_t SerialMain::SenderPackSolve(uint8_t *data, uint16_t data_length,
									 uint16_t cmd_id, uint8_t *send_buf)
{
	uint16_t index = 0;//游标（用 uint16_t，别用 uint8_t，防止溢出）

	frame_send_header_.sof = io::HEADER_SOF;
	frame_send_header_.data_length = data_length;//正文长度，= sizeof(io::GimbalCtrlData) = 31
	frame_send_header_.seq++;//序列号+1，电控可用来检测丢包

	// 帧头 CRC8：对帧头前 4 字节（sof+data_length+seq）计算，填到第 5 字节
	Append_CRC8_Check_Sum((uint8_t *)&frame_send_header_, FRAME_HEAD_LEN);

	memcpy(send_buf, &frame_send_header_, FRAME_HEAD_LEN);//帧头写入发送缓冲区
	index += FRAME_HEAD_LEN;

	memcpy(send_buf + index, &cmd_id, sizeof(uint16_t));//消息类型写入
	index += sizeof(uint16_t);

	memcpy(send_buf + index, data, data_length);//正文写入
	index += data_length;

	// 整帧 CRC16：覆盖 帧头+cmd_id+正文（data_length + 7 字节），
	// 结果小端（低字节在前）写在正文后面 2 字节
	Append_CRC16_Check_Sum(send_buf, data_length + 9);

	index += FRAME_CRC16_LEN;

	// 帧尾 0x0D 0x0A（protocol_new.hpp 的 MsgEndInfo）
	// 25 赛季发送端没加帧尾也碰巧能用，但协议里定义了，加上更稳：
	// 按帧头+长度解析的电控会自动跳过这 2 字节，按帧尾收帧的电控正好需要它
	send_buf[index++] = io::END1_SOF;
	send_buf[index++] = io::END2_SOF;

	return index;//整帧总长度 = data_length + 11
}
