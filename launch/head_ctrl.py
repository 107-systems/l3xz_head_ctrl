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
        {'head_topic': 'cmd_vel_head'},
        {'head_topic_deadline_ms': 100},
        {'head_topic_liveliness_lease_duration': 1000},
        {'pan_actual_angle_topic': 'head/pan/angle/actual'},
        {'pan_actual_angle_topic_deadline_ms': 100},
        {'pan_actual_angle_topic_liveliness_lease_duration': 1000},
        {'tilt_actual_angle_topic': 'head/tilt/angle/actual'},
        {'tilt_actual_angle_topic_deadline_ms': 100},
        {'tilt_actual_angle_topic_liveliness_lease_duration': 1000},
        {'pan_initial_angle_deg': 180.0},
        {'pan_min_angle_deg': 180.0 - 25.0},
        {'pan_max_angle_deg': 180.0 + 25.0},
        {'tilt_initial_angle_deg': 90.0},
        {'tilt_min_angle_deg': 90.0 - 45.0},
        {'tilt_max_angle_deg': 90.0 + 45.0},
      ],
    )
  ])
