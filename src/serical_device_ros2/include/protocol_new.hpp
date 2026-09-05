#ifndef IO__PROTOCOL_HJ_HPP
#define IO__PROTOCOL_HJ_HPP

// 电控（hj）给出的正式协议，2026-08-30 起为唯一基准
// 注意和旧版 protocol_new 的区别：
//   1. 视觉->电控控制帧：cmd 0x0102，正文 GimbalCtrlData 31 字节（多了 main_yaw、fire_mode，末尾三字节顺序不同）
//   2. 电控->视觉数据帧：cmd 0x0104（旧版是 0x0105），正文 VisionData 49 字节（没有 mode 字段，多了 main_yaw）
//   3. 旧的 RobotCtrlData / MsgEndInfo 已废弃

#include <cstdint>

namespace io
{

constexpr uint8_t HEADER_SOF = 0xA5;
constexpr uint8_t END1_SOF = 0x0D;
constexpr uint8_t END2_SOF = 0x0A;

constexpr uint16_t CHASSIS_CTRL_CMD_ID = 0x0101;
constexpr uint16_t VISION_CTRL_CMD_ID = 0x0102;
constexpr uint16_t RECEIVE_COMPETITION_INFO_CMD_ID = 0x0103;
constexpr uint16_t VISION_ID = 0x0104;
constexpr uint16_t REFEREE_CMD_ID = 0x0105;
constexpr uint16_t MAIN_YAW_CTRL_CMD_ID = 0x0106;
constexpr uint16_t BASE_ATTACK_CMD_ID = 0x0107;
constexpr uint16_t ROBOT_NAV_INFO_ID = 0x0108;
constexpr uint16_t ROBOT_POSE_ID = 0x0109;
constexpr uint16_t MODECONTROL_ID = 0x010A;
constexpr uint16_t SENTRY_POSTURE_CMD_ID = 0x010B;
constexpr uint16_t VISION_TARGET_INFO_CMD_ID = 0x010C;

struct __attribute__((packed)) FrameHeader
{
  uint8_t sof = HEADER_SOF;
  uint16_t data_length = 0;
  uint8_t seq = 0;
  uint8_t crc8 = 0;
};

struct __attribute__((packed)) ChassisCtrlData
{
  float vx = 0.0f;
  float vy = 0.0f;
};

struct __attribute__((packed)) VisionData
{
  float yaw = 0.0f;
  float yaw_vel = 0.0f;
  float main_yaw = 0.0f;
  float pitch = 0.0f;
  float pitch_vel = 0.0f;
  float roll = 0.0f;
  float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z
  float shoot_speed = 0.0f;
  uint16_t id = 0;
  uint16_t bullet_count = 0;
  uint8_t game_progress = 0;
};

struct __attribute__((packed)) GimbalCtrlData
{
  // 原 VISION_CTRL_CMD_ID payload:
  // float main_yaw = 0.0f;
  // float pitch = 0.0f;
  // float yaw = 0.0f;
  // int8_t fire_command = 0;
  // int8_t fire_mode = 1;
  // int8_t target_lock = 50;  // 49: lock, 50: unlock

  float yaw = 0.0f;
  float yaw_vel = 0.0f;
  float yaw_acc = 0.0f;
  float pitch = 0.0f;
  float pitch_vel = 0.0f;
  float pitch_acc = 0.0f;
  float main_yaw = 0.0f;
  int8_t fire_command = 0;
  int8_t fire_mode = 1;
  int8_t target_lock = 50;  // 49: lock, 50: unlock
};

struct __attribute__((packed)) AttackBaseData
{
  float yaw = 0.0f;
  int8_t flag = 0;
};

struct __attribute__((packed)) MainYawCtrlData
{
  float yaw = 0.0f;
};

struct __attribute__((packed)) RobotPoseData
{
  float robot_pos_x = 0.0f;
  float robot_pos_y = 0.0f;
};

struct __attribute__((packed)) CompetitionInfoData
{
  uint8_t game_state = 0;
  uint16_t stage_remain_time = 0;
  uint32_t event_data = 0;
  uint16_t our_outpost_hp = 0;
  uint16_t enemy_outpost_hp = 0;
  uint16_t remain_bullet = 0;
  uint16_t enemy_sentry_hp = 0;
  uint16_t our_sentry_hp = 0;
  uint16_t our_base_hp = 0;
  uint8_t first_blood = 0;
  float target_position_x = 0.0f;
  float target_position_y = 0.0f;
  int8_t is_target_active = 0;
  uint16_t enemy_hero_hp = 0;
  uint16_t enemy_engineer_hp = 0;
  uint16_t enemy_infantry_3_hp = 0;
  uint16_t enemy_infantry_4_hp = 0;
  uint16_t enemy_base_hp = 0;
  uint16_t our_hero_hp = 0;
  uint16_t our_engineer_hp = 0;
  uint16_t our_infantry_3_hp = 0;
  uint16_t our_infantry_4_hp = 0;
  uint16_t remain_energy = 0;
  uint16_t sentry_exchange_ammo_num = 0;
  uint8_t sentry_remote_ammo_times = 0;
  uint8_t sentry_remote_hp_times = 0;
  uint8_t sentry_can_free_resurrect = 0;
  uint8_t sentry_can_pay_resurrect = 0;
  uint16_t sentry_buy_resurrect_gold = 0;
  uint8_t sentry_is_out_of_combat = 0;
  uint16_t sentry_team_ammo_exchange_left = 0;
  uint8_t sentry_posture = 0;
  uint8_t sentry_can_activate_rune = 0;
};

struct __attribute__((packed)) ModeControlData
{
  uint16_t chassis_gyro = 0;
  uint8_t patrol_mode = 0;
  uint8_t power_mode = 0;
};

struct __attribute__((packed)) SentryPostureCmdData
{
  uint8_t posture_cmd = 0;
};

constexpr uint8_t TARGET_STATE_NONE = 0;
constexpr uint8_t TARGET_STATE_TRACKING = 1;
constexpr uint8_t TARGET_STATE_OMNI_DETECTED = 2;

struct __attribute__((packed)) TargetIdentifyInfo
{
  uint8_t state = 0;
  uint8_t armor_name = 0;
  float distance = 0.0f;
};

}  // namespace io

#endif  // IO__PROTOCOL_HJ_HPP
