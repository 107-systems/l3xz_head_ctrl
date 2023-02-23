from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_head_ctrl',
      namespace='l3xz',
      executable='l3xz_head_ctrl_node',
      name='l3xz_head_ctrl',
      output='screen',
      emulate_tty=True,
      parameters=[
        {'pan_initial_angle_deg': 180.0},
        {'pan_min_angle_deg': 160.0},
        {'pan_max_angle_deg': 200.0},
        {'tilt_initial_angle_deg': 180.0},
        {'tilt_min_angle_deg': 160.0},
        {'tilt_max_angle_deg': 200.0},
      ],
    )
  ])
