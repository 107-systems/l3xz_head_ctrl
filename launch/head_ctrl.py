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
      parameters=[]
    )
  ])
