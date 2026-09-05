#include <rclcpp/rclcpp.hpp>
#include "serial_main.h"
// #include "robot_status.h"
// #include "robot_struct.h"
#include "auto_aim_interfaces/msg/robot_ctrl.hpp"
#include "auto_aim_interfaces/msg/vision.hpp"

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

    RCLCPP_INFO(this->get_logger(), "-- RobotCtrlSub Node Started --");
  }

private:
  // void robotCtrlSend(const auto_aim_interfaces::msg::RobotCtrl::SharedPtr msg)
  // void robotCtrlSend(const auto_aim_interfaces::msg::RobotCtrl &msg) const 
  void robotCtrlSend(const auto_aim_interfaces::msg::RobotCtrl::ConstSharedPtr& msg)
  {
    double fire = static_cast<double>(msg->fire_command);
    double mode = static_cast<double>(msg->target_lock);
    vdata = {msg->pitch, msg->yaw, fire, mode};
    serial.SenderMain(vdata);    
    // std::cout<<"---------- ROBOT CTRL SEND ----  "<<" yaw: "<<(double)msg->yaw<<std::endl;
  }
//   void robotCtrlSend(const auto_aim_interfaces::msg::RobotCtrl::ConstSharedPtr& msg)
// {
//   RCLCPP_INFO(this->get_logger(), "--- into robotCtrlSend ---");
//   // RCLCPP_INFO(this->get_logger(), "[RECV] vx: %.2f, vy: %.2f, vw: %.2f", msg->vx, msg->vy, msg->vw);
//   // RCLCPP_INFO(this->get_logger(), "[RECV] yaw: %.2f, pitch: %.2f", msg->yaw, msg->pitch);
//   // RCLCPP_INFO(this->get_logger(), "[RECV] fire: %d, lock: %d", msg->fire_command, msg->target_lock);
// }

  SerialMain serial;
  std::vector<double> vdata{4};
  rclcpp::Subscription<auto_aim_interfaces::msg::RobotCtrl>::SharedPtr subscription_;
};


}  // namespace rm_auto_aim
// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<rm_auto_aim::RobotCtrlSub>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

#include "rclcpp_components/register_node_macro.hpp"
// 注册为组件
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RobotCtrlSub)
