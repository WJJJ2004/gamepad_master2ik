from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gamepad_master2ik',
            executable='gamepad_master2ik_node',
            name='gamepad_master2ik_node',
            output='screen',
            parameters=[
                {
                    'publish_rate_hz': 50.0,

                    'x_max': 25.0,
                    'x_min': -19.0,
                    'y_max': 17.0,
                    'y_min': -12.0,
                    'yaw_max': 5.0,
                    'yaw_min': -5.0,

                    'x_step': 0.35,
                    'y_step': 0.30,
                    'yaw_step': 0.08,

                    'x_return_step': 0.25,
                    'y_return_step': 0.20,
                    'yaw_return_step': 0.06,

                    'deadband_x': 0.05,
                    'deadband_y': 0.05,
                    'deadband_yaw': 0.05,

                    'flag': 1.0,
                    'cp_flag': 0.0,

                    'vel_scale_x': 1.0,
                    'vel_scale_y': 1.0,
                    'vel_scale_rot': 1.0,

                    'device_path': '',
                }
            ]
        )
    ])
