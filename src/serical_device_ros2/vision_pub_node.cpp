#include <rclcpp/rclcpp.hpp>
#include "serial_main.h"
// #include "robot_status.h"
// #include "robot_struct.h"
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

    RCLCPP_INFO(this->get_logger(), "--- VisionPub Node Started ---");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&VisionPub::timer_callback, this));
  }

private:
  bool Get_data;
  SerialMain serial;
  rclcpp::Publisher<auto_aim_interfaces::msg::Vision>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback()
  {
    Get_data = serial.ReceiverMain();
    if (Get_data)
    {
      auto vision_t = std::make_shared<auto_aim_interfaces::msg::Vision>();

      vision_t->header.frame_id = "vision";
      vision_t->header.stamp = this->now();
      vision_t->id = serial.vision_msg_.id;
      vision_t->mode = serial.vision_msg_.mode;
      vision_t->pitch = serial.vision_msg_.pitch;
      vision_t->yaw = serial.vision_msg_.yaw;
      vision_t->roll = serial.vision_msg_.roll;

      vision_t->quaternion.resize(4);
      for (int i = 0; i < 4; i++)
      {
        vision_t->quaternion[i] = serial.vision_msg_.quaternion[i];
      }

      vision_t->shoot = serial.vision_msg_.shoot;

      publisher_->publish(*vision_t);
    }
  }
};

}  // namespace rm_auto_aim
// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<rm_auto_aim::VisionPub>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

// 注册为组件
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::VisionPub)