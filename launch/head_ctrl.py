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
      remappings=[
        ('/dynamixel/servo_7/angle/actual',            '/l3xz/head/pan/angle/actual'),
        ('/dynamixel/servo_7/angle/target',            '/l3xz/head/pan/angle/target'),
        ('/dynamixel/servo_7/angular_velocity/target', '/l3xz/head/pan/angular_velocity/target'),
        ('/dynamixel/servo_7/mode/set',                '/l3xz/head/pan/mode/set'),
        ('/dynamixel/servo_8/angle/actual',            '/l3xz/head/tilt/angle/actual'),
        ('/dynamixel/servo_8/angle/target',            '/l3xz/head/tilt/angle/target'),
        ('/dynamixel/servo_8/angular_velocity/target', '/l3xz/head/tilt/angular_velocity/target'),
        ('/dynamixel/servo_8/mode/set',                '/l3xz/head/tilt/mode/set'),
      ]
    )
  ])
