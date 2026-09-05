# 下位机(STM32) -> 上位机(视觉) 数据
# 字段与协议文件 protocol_new.hpp 中 io::VisionData 一一对应
std_msgs/Header header

# 机器人 id
uint16 id
# 模式：33 = 自瞄
uint16 mode

# 云台姿态（单位：度，由下位机给出）
float32 yaw
float32 yaw_vel
float32 pitch
float32 pitch_vel
float32 roll

# 姿态四元数 [w, x, y, z]
float32[4] quaternion

# 弹速（m/s）
float32 shoot_speed
# 剩余弹量
uint16 bullet_count
# 比赛进度
uint8 game_progress
