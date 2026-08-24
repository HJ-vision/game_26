#ifndef IO__PROTOCOL_HJ_HPP
#define IO__PROTOCOL_HJ_HPP

#include <cstdint>

namespace io
{

constexpr uint8_t HEADER_SOF = 0xA5;
constexpr uint8_t END1_SOF = 0x0D;
constexpr uint8_t END2_SOF = 0x0A;

constexpr uint16_t CHASSIS_ODOM_CMD_ID = 0x0101;
constexpr uint16_t CHASSIS_CTRL_CMD_ID = 0x0102;
constexpr uint16_t RGB_ID = 0x0103;
constexpr uint16_t RC_ID = 0x0104;
constexpr uint16_t VISION_ID = 0x0105;

struct __attribute__((packed)) FrameHeader
{
  uint8_t sof = HEADER_SOF;
  uint16_t data_length = 0;
  uint8_t seq = 0;
  uint8_t crc8 = 0;
};

struct __attribute__((packed)) VisionData
{
  uint16_t id = 0;
  uint16_t mode = 0;  // 33: auto aim
  float yaw = 0.0f;
  float yaw_vel = 0.0f;
  float pitch = 0.0f;
  float pitch_vel = 0.0f;
  float roll = 0.0f;
  float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
  float shoot_speed = 0.0f;
  uint16_t bullet_count = 0;
  uint8_t game_progress = 0;
};

struct __attribute__((packed)) RobotCtrlData
{
  float yaw = 0.0f;
  float yaw_vel = 0.0f;
  float yaw_acc = 0.0f;
  float pitch = 0.0f;
  float pitch_vel = 0.0f;
  float pitch_acc = 0.0f;
  int8_t target_lock = 50;  // 49: lock, 50: unlock
  int8_t fire_command = 0;
};

struct __attribute__((packed)) MsgEndInfo
{
  uint8_t end1 = END1_SOF;
  uint8_t end2 = END2_SOF;
};

}  // namespace io

#endif  // IO__PROTOCOL_HJ_HPP
