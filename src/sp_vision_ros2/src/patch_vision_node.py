#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
给 sp_vision_ros2 的 vision_node.cpp 打补丁：
  1) 新增 freeze_gimbal 参数：开启时只给云台发"保持当前角"，云台冻结不动；
  2) 新增 /aim_solved 话题：始终发布求解出的目标角(度)+装甲板图像x+有效性+装甲板数，
     方便"云台不转"也能读数诊断。

用法（在机器人小电脑上，src/sp_vision_ros2/src/ 目录下执行）：
  python3 patch_vision_node.py vision_node.cpp
幂等：已打过补丁会提示并直接退出，不会重复插入。
"""
import sys
import os

SRC_DEFAULT = "vision_node.cpp"

markers = [
    "#include <std_msgs/msg/float64_multi_array.hpp>",
    "freeze_gimbal_ = declare_parameter",
    '"/aim_solved"',
    "n_armors_ = static_cast",
    "freeze_gimbal_ && imu_received_",
    "solved_pub_->publish",
    "bool freeze_gimbal_ = false",
    "solved_pub_;",
    "double plate_x_ = 0.0",
]

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else SRC_DEFAULT
    if not os.path.isfile(path):
        print("找不到文件:", path)
        sys.exit(1)
    with open(path, "r", encoding="utf-8") as f:
        s = f.read()

    if all(m in s for m in markers):
        print("已经打过补丁，无需重复操作。")
        return

    # A. include
    a = "    #include <std_msgs/msg/header.hpp>\n"
    b = "    #include <std_msgs/msg/header.hpp>\n    #include <std_msgs/msg/float64_multi_array.hpp>\n"
    if "#include <std_msgs/msg/float64_multi_array.hpp>" not in s:
        assert a in s, "找不到 include 锚点"
        s = s.replace(a, b, 1)

    # B. param
    a = '    auto_fire_ = declare_parameter<bool>("auto_fire", true);\n'
    b = '    auto_fire_ = declare_parameter<bool>("auto_fire", true);\n    freeze_gimbal_ = declare_parameter<bool>("freeze_gimbal", false);\n'
    if "freeze_gimbal_ = declare_parameter" not in s:
        assert a in s, "找不到 param 锚点"
        s = s.replace(a, b, 1)

    # C. publisher
    a = '    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/aim_debug_image", 1);\n'
    b = '    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/aim_debug_image", 1);\n    solved_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/aim_solved", 10);\n'
    if '"/aim_solved"' not in s:
        assert a in s, "找不到 publisher 锚点"
        s = s.replace(a, b, 1)

    # D. image_cb capture (只改带 // 主流水线 注释的那个)
    a = "    // 主流水线\n    auto armors = detector_->detect(bgr, frame_count_++);\n    auto targets = tracker_->track(armors, std::chrono::steady_clock::now());\n"
    b = "    // 主流水线\n    auto armors = detector_->detect(bgr, frame_count_++);\n    auto targets = tracker_->track(armors, std::chrono::steady_clock::now());\n    n_armors_ = static_cast<int>(armors.size());\n    plate_x_ = armors.empty() ? 0.0 : armors.front().center.x;\n"
    if "n_armors_ = static_cast" not in s:
        assert a in s, "找不到 image_cb 锚点"
        s = s.replace(a, b, 1)

    # E. publish_ctrl freeze 分支
    a = "    auto msg = std::make_unique<auto_aim_interfaces::msg::RobotCtrl>();\n\n    if (active && !no_target && cmd.control) {\n"
    b = "    auto msg = std::make_unique<auto_aim_interfaces::msg::RobotCtrl>();\n\n    if (freeze_gimbal_ && imu_received_) {\n      // 冻结云台：只发当前 IMU 角度（保持不动），方便读数诊断\n      msg->yaw = latest_yaw_;\n      msg->pitch = latest_pitch_;\n      msg->target_lock = 50;\n      msg->fire_command = 0;\n    } else if (active && !no_target && cmd.control) {\n"
    if "freeze_gimbal_ && imu_received_" not in s:
        assert a in s, "找不到 publish_ctrl 锚点"
        s = s.replace(a, b, 1)

    # F. /aim_solved 发布
    a = "    ctrl_pub_->publish(std::move(msg));\n  }\n"
    b = "    // 诊断 topic：始终发布\"求解出的目标角\"（不受 freeze 影响），方便不转云台也能读数\n    {\n      std_msgs::msg::Float64MultiArray s;\n      s.data = {\n        static_cast<double>(cmd.yaw * 180.0 / M_PI),\n        static_cast<double>(cmd.pitch * 180.0 / M_PI),\n        plate_x_,\n        static_cast<double>(!no_target && cmd.control ? 1.0 : 0.0),\n        static_cast<double>(n_armors_)};\n      solved_pub_->publish(s);\n    }\n\n    ctrl_pub_->publish(std::move(msg));\n  }\n"
    if "solved_pub_->publish" not in s:
        assert a in s, "找不到 publish 锚点"
        s = s.replace(a, b, 1)

    # G. 成员声明
    a = "  bool auto_fire_;\n  bool debug_;\n  bool show_;\n"
    b = "  bool auto_fire_;\n  bool freeze_gimbal_ = false;\n  bool debug_;\n  bool show_;\n"
    if "bool freeze_gimbal_ = false" not in s:
        assert a in s, "找不到成员 bool 锚点"
        s = s.replace(a, b, 1)

    a = "  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;\n"
    b = "  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;\n  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr solved_pub_;\n"
    if "solved_pub_;" not in s:
        assert a in s, "找不到成员 publisher 锚点"
        s = s.replace(a, b, 1)

    a = "  // 状态\n  int frame_count_ = 0;\n  bool imu_received_ = false;\n"
    b = "  // 状态\n  int frame_count_ = 0;\n  double plate_x_ = 0.0;\n  int n_armors_ = 0;\n  bool imu_received_ = false;\n"
    if "double plate_x_ = 0.0" not in s:
        assert a in s, "找不到成员 状态 锚点"
        s = s.replace(a, b, 1)

    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(s)
    print("补丁已应用 ->", path)
    print("记得：colcon build --packages-select sp_vision_ros2")

if __name__ == "__main__":
    main()
