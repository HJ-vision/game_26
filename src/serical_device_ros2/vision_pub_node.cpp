#include <rclcpp/rclcpp.hpp>
#include "serial_main.h"
// 注意：串口收发现在都由这一个节点（vision_pub）负责！
// 原因：/dev/robomaster 这个串口只允许被一个进程打开。
// 之前 vision_pub 和 robot_ctrl 两个节点各开一次串口，
// 第二次 open 时的 tcflush 会清空缓冲丢字节，帧被拆散 -> CRC 校验失败。
// 所以把“订阅 /Robot_ctrl_data 并发送串口”也搬进本节点，
// robot_ctrl_node 现在只是个调试打印节点，不再碰串口。
#include "auto_aim_interfaces/msg/robot_ctrl.hpp"
#include "auto_aim_interfaces/msg/vision.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace rm_auto_aim
{
class VisionPub : public rclcpp::Node
{
public:
  explicit VisionPub(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("vision_pub", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    publisher_ = this->create_publisher<auto_aim_interfaces::msg::Vision>("/Vision_data", 10);

    // 订阅自瞄解算节点的控制输出，转发到串口
    ctrl_sub_ = this->create_subscription<auto_aim_interfaces::msg::RobotCtrl>(
      "/Robot_ctrl_data", 10,
      std::bind(&VisionPub::ctrlCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "--- VisionPub Node Started (serial RX + TX) ---");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&VisionPub::timer_callback, this));
    // timer 和 ctrl_sub_ 用默认的互斥回调组，两个回调不会同时执行，
    // 串口的 read/write 已全部非阻塞化（O_NONBLOCK + VMIN=0），
    // 所以无论是读不到数据还是写缓冲满，回调都会立刻返回，谁也卡不死谁。
  }

private:
  bool Get_data = false;
  SerialMain serial;//全节点唯一一个串口实例
  rclcpp::Publisher<auto_aim_interfaces::msg::Vision>::SharedPtr publisher_;
  rclcpp::Subscription<auto_aim_interfaces::msg::RobotCtrl>::SharedPtr ctrl_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 定时收：读串口 -> 校验 CRC -> 发布 /Vision_data
  void timer_callback()
  {
    Get_data = serial.ReceiverMain();
    if (Get_data)
    {
      auto vision_t = std::make_shared<auto_aim_interfaces::msg::Vision>();

      vision_t->header.frame_id = "vision";
      vision_t->header.stamp = this->now();
      vision_t->id = serial.vision_msg_.id;
      vision_t->mode = 0;//protocol_hj 的 VisionData 里没有 mode 字段（按钮状态不经过串口），保持 0
      vision_t->pitch = serial.vision_msg_.pitch;
      vision_t->yaw = serial.vision_msg_.yaw;
      vision_t->yaw_vel = serial.vision_msg_.yaw_vel;
      vision_t->pitch_vel = serial.vision_msg_.pitch_vel;
      vision_t->roll = serial.vision_msg_.roll;

      for (int i = 0; i < 4; i++)
      {
        vision_t->quaternion[i] = serial.vision_msg_.quaternion[i];
      }

      vision_t->shoot_speed = serial.vision_msg_.shoot_speed;
      vision_t->bullet_count = serial.vision_msg_.bullet_count;
      vision_t->game_progress = serial.vision_msg_.game_progress;

      publisher_->publish(*vision_t);
    }
  }

  // 订阅发：/Robot_ctrl_data -> 打协议帧 -> 写串口
  // 字段顺序先按 RobotCtrl.msg 排，SenderMain 内部再转成 protocol_hj 的 GimbalCtrlData
  void ctrlCallback(const auto_aim_interfaces::msg::RobotCtrl::ConstSharedPtr & msg)
  {
    std::vector<double> vdata = {
      msg->yaw,
      msg->yaw_vel,
      msg->yaw_acc,
      msg->pitch,
      msg->pitch_vel,
      msg->pitch_acc,
      static_cast<double>(msg->target_lock),
      static_cast<double>(msg->fire_command)
    };
    serial.SenderMain(vdata);
  }
};

}  // namespace rm_auto_aim

// 注册为组件
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::VisionPub)
