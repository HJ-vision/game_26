#include <rclcpp/rclcpp.hpp>
#include "auto_aim_interfaces/msg/robot_ctrl.hpp"
#include "auto_aim_interfaces/msg/vision.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// ⚠️ 本节点已经不负责发串口了！
// 串口发送已并入 vision_pub 节点（同一个进程独占 /dev/robomaster，
// 避免两个节点各开一次串口导致 tcflush 丢字节、CRC 校验失败）。
// 保留本节点只做一件事：订阅 /Robot_ctrl_data 并打印，
// 用来在终端里观察自瞄解算到底输出了什么（调试用）。
// 实机跑的时候不需要启动它。

namespace rm_auto_aim
{

class RobotCtrlSub : public rclcpp::Node
{
public:
  explicit RobotCtrlSub(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robot_ctrl", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    subscription_ = this->create_subscription<auto_aim_interfaces::msg::RobotCtrl>(
      "/Robot_ctrl_data", 10,
      std::bind(&RobotCtrlSub::robotCtrlSend, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "-- RobotCtrlSub Node Started (debug echo only, NO serial) --");
  }

private:
  void robotCtrlSend(const auto_aim_interfaces::msg::RobotCtrl::ConstSharedPtr & msg)
  {
    // 不再打印回显：静止锁定时指令恒定，反复输出同一个值没有意义。
    // 需要观察时用 ros2 topic echo /Robot_ctrl_data 即可。
    (void)msg;
  }

  rclcpp::Subscription<auto_aim_interfaces::msg::RobotCtrl>::SharedPtr subscription_;
};

}  // namespace rm_auto_aim

// 注册为组件
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RobotCtrlSub)
