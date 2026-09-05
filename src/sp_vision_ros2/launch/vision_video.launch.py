# ============================================================
# vision_video.launch.py — 离线视频验证模式
# 用视频文件跑完整流水线（detect -> track -> aim），不需要相机/串口。
# 结果发布到 /aim_debug_image，控制指令发布到 /Robot_ctrl_data。
#
# 用法（video_path 传绝对路径）：
#   ros2 launch sp_vision_ros2 vision_video.launch.py video_path:=/home/user/test.mp4
#   可选：show:=true 弹窗实时预览（需要显示器）
# ============================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


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
                'mode': 'video',
                'video_path': LaunchConfiguration('video_path'),
                'bullet_speed': 23.0,
                'auto_fire': True,
                'debug': True,
                'debug_ekf_viz': True,   # 叠加 EKF 预测装甲板位置可视化
                'show': LaunchConfiguration('show', default=False),
                'save_path': LaunchConfiguration('save_path', default=''),
            }],
        ),
    ])
