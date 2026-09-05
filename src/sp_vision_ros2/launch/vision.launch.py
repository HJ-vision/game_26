# ============================================================
# vision.launch.py — 实机模式
# 前提：海康相机节点已发布 /image_raw，串口节点已发布 /Vision_data
# 用法：
#   ros2 launch sp_vision_ros2 vision.launch.py
# ============================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sp_vision_ros2')

    return LaunchDescription([
        Node(
            package='sp_vision_ros2',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[{
                'config_path': os.path.join(pkg_share, 'configs', 'standard3.yaml'),
                'mode': 'camera',
                'auto_fire': True,
                'debug': True,
                'debug_ekf_viz': True,   # 叠加 EKF 预测装甲板位置可视化
                'debug_force_yaw': False,  # 关闭强制 yaw=1.0 调试，跑真实自瞄
            }],
        ),
    ])
