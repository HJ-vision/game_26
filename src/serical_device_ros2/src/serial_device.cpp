/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "serial_device.h"
#include <iostream>
#include <cerrno>

//namespace robomaster {
SerialDevice::SerialDevice(std::string port_name,
						   int baudrate) :
		port_name_(port_name),
		baudrate_(baudrate),
		data_bits_(8),
		parity_bits_('N'),
		stop_bits_(1) {}

SerialDevice::~SerialDevice() {
	CloseDevice();
}

// 初始化
bool SerialDevice::Init() {
	
	std::cout << "Attempting to open device " << port_name_
			  << " with baudrate " << baudrate_ << std::endl;
	if (port_name_.c_str() == nullptr) {
		port_name_ = "/dev/ttyUSB0";
	}
	if (OpenDevice() && ConfigDevice()) {
		FD_ZERO(&serial_fd_set_);
		FD_SET(serial_fd_, &serial_fd_set_);
		std::cout << "Serial started successfully." << std::endl;
		return true;
	} else {
		std::cerr << "Failed to start serial " << port_name_ << std::endl;
		CloseDevice();
		return false;
	}
}

// 打开设备
bool SerialDevice::OpenDevice() {

	// 全平台统一非阻塞打开：read/write 绝不能把 ROS 回调（单线程 executor）卡死。
	// 原来的 #ifdef __arm__ / #elif __x86_64__ 在 Jetson（aarch64）上一个都不命中，
	// 走了 #else 的纯阻塞分支；配合 VMIN=18，电控遥测一停 read() 就无限阻塞，
	// 整个串口节点冻结（Vision_data 断流 + 控制帧发不出去）。
	serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (serial_fd_ < 0) {
		std::cerr << "Cannot open device "
				  << port_name_ << std::endl;
		return false;
	}

	return true;
}

// 关闭通讯协议接口
bool SerialDevice::CloseDevice() {
	close(serial_fd_);
	serial_fd_ = -1;
	return true;
}

// 配置设备
bool SerialDevice::ConfigDevice() {
	int st_baud[] = {B4800, B9600, B19200, B38400,
					 B57600, B115200, B230400, B921600};
	int std_rate[] = {4800, 9600, 19200, 38400, 57600, 115200,
					  230400, 921600, 1000000, 1152000, 3000000};
	int i, j;
	/* save current port parameter */
	if (tcgetattr(serial_fd_, &old_termios_) != 0) {
		std::cerr << "fail to save current port" << std::endl;
		return false;
	}
	memset(&new_termios_, 0, sizeof(new_termios_));
	
	/* config the size of char */
	new_termios_.c_cflag |= CLOCAL | CREAD;
	new_termios_.c_cflag &= ~CSIZE;
	
	/* config data bit */
	switch (data_bits_) {
		case 7:
			new_termios_.c_cflag |= CS7;
			break;
		case 8:
			new_termios_.c_cflag |= CS8;
			break;
		default:
			new_termios_.c_cflag |= CS8;
			break; //8N1 default config
	}
	/* config the parity bit */
	switch (parity_bits_) {
		/* odd */
		case 'O':
		case 'o':
			new_termios_.c_cflag |= PARENB;
			new_termios_.c_cflag |= PARODD;
			break;
			/* even */
		case 'E':
		case 'e':
			new_termios_.c_cflag |= PARENB;
			new_termios_.c_cflag &= ~PARODD;
			break;
			/* none */
		case 'N':
		case 'n':
			new_termios_.c_cflag &= ~PARENB;
			break;
		default:
			new_termios_.c_cflag &= ~PARENB;
			break; //8N1 default config
	}
	/* config baudrate */
	j = sizeof(std_rate) / 4;
	for (i = 0; i < j; ++i) {
		if (std_rate[i] == baudrate_) {
			/* set standard baudrate */
			cfsetispeed(&new_termios_, st_baud[i]);
			cfsetospeed(&new_termios_, st_baud[i]);
			break;
		}
	}
	/* config stop bit */
	if (stop_bits_ == 1)
		new_termios_.c_cflag &= ~CSTOPB;
	else if (stop_bits_ == 2)
		new_termios_.c_cflag |= CSTOPB;
	else
		new_termios_.c_cflag &= ~CSTOPB; //8N1 default config

/* config waiting time & min number of char */
	// VMIN=0/VTIME=0：读到多少算多少，没有数据立即返回。
	// （原来是 VMIN=18，凑不够 18 字节就阻塞，遥测一停节点就冻死）
	new_termios_.c_cc[VMIN] = 0;
	new_termios_.c_cc[VTIME] = 0;
	
	/* using the raw data mode */
	new_termios_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	new_termios_.c_oflag &= ~OPOST;
	
	/* flush the hardware fifo */
	tcflush(serial_fd_, TCIFLUSH);
	
	/* activite the configuration */
	if ((tcsetattr(serial_fd_, TCSANOW, &new_termios_)) != 0) {
		std::cerr << "failed to activate serial configuration" << std::endl;
		return false;
	}

	// Linux 的 tcsetattr（TCSETS ioctl）会把 fd 上的 O_NONBLOCK 标志清掉，
	// 必须在配置完成后再加回来，否则上面 open 时的非阻塞就白开了
	int fl = fcntl(serial_fd_, F_GETFL, 0);
	if (fl >= 0) {
		fcntl(serial_fd_, F_SETFL, fl | O_NONBLOCK);
	}
	return true;

}

int SerialDevice::Read(uint8_t *buf, int len) {
	if (NULL == buf || len <= 0) {
		return 0;
	}

	int ret = read(serial_fd_, buf, len);
	if (ret > 0) {
		return ret;
	}
	if (ret == 0) {
		return 0;	// VMIN=0 模式：当前没有数据，不是断连
	}
	if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
		return 0;	// 非阻塞模式：没有数据，正常情况
	}

	// 真正的读错误（串口拔线等）：限频打印防止 100Hz 刷屏。
	// 注意绝对不能在这里做阻塞重连——本函数跑在 ROS 回调里，
	// 卡住它就是卡住整个单线程 executor（老代码的教训）。
	static int err_count = 0;
	if (++err_count % 100 == 1) {
		std::cerr << "Serial read error: " << strerror(errno)
				  << " (err_count=" << err_count << ")" << std::endl;
	}
	return -1;
}

// 发送数据函数
int SerialDevice::Write(const uint8_t *buf, int len) {
	int ret = write(serial_fd_, buf, len);
	if (ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
		// 非阻塞 write 遇到 EAGAIN（TX 缓冲满）时这帧被丢弃：
		// 42 字节@15Hz 远填不满缓冲，只有下位机彻底失联才会发生，丢了也没用
		static int err_count = 0;
		if (++err_count % 100 == 1) {
			std::cerr << "Serial write error: " << strerror(errno)
					  << " (err_count=" << err_count << ")" << std::endl;
		}
	}
	return ret;
}
//}
