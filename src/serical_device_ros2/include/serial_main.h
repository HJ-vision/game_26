#ifndef ROBOMASTER_ROBOT_H
#define ROBOMASTER_ROBOT_H

#include <iostream>
#include <thread>
#include <vector>
#include "serial_device.h"
#include "protocol_new.hpp"
#include "crc.h"
#include <memory> 
class SerialMain {
public:
	SerialMain(std::string device_path = "/dev/robomaster");
	
	~SerialMain() = default;
	
	void SenderMain(const std::vector<double> &vdata);          // 发送数据
	
	bool CommInit();
	
	bool ReceiverMain();                                        // 读取数据
	
	void SearchFrameSOF(uint8_t *frame, uint16_t total_len);
	
	uint16_t ReceiveDataSolve(uint8_t *frame);
	
	uint16_t SenderPackSolve(uint8_t *data, uint16_t data_length,
							 uint16_t cmd_id, uint8_t *send_buf);
	io::VisionData vision_msg_;

private:
	
	//! Device Information and Buffer Allocation
	std::string device_path_;
	std::shared_ptr<SerialDevice> device_ptr_;
	std::unique_ptr<uint8_t[]> recv_buff_;
	std::unique_ptr<uint8_t[]> send_buff_;
	const unsigned int BUFF_LENGTH = 512;
	
	//! Frame Information
	io::FrameHeader frame_receive_header_;
	io::FrameHeader frame_send_header_;
	uint8_t seq_counter_ = 0;
	
	/** @brief specific protocol data are defined here
	 *         xxxx_info_t is defined in
	 */
	
	io::RobotCtrlData robot_ctrl;
};
//}

#endif // ROBOMASTER_ROBOT_H
