# 上位机(视觉) -> 下位机(STM32) 控制数据
# 字段与协议文件 protocol_new.hpp 中 io::RobotCtrlData 一一对应
std_msgs/Header header

# 云台目标角度（单位：度）
float32 yaw
float32 yaw_vel
float32 yaw_acc
float32 pitch
float32 pitch_vel
float32 pitch_acc

# 锁敌：49 = 锁定，50 = 未锁定
int8 target_lock
# 开火指令：0 = 不发射，1 = 发射
int8 fire_command
