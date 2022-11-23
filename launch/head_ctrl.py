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
      parameters=[
          {'serial_port' : '"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NNZ55-if00-port0"'},
          {'serial_port_baudrate': 115200},
          {'pan_servo_id': 7},
          {'tilt_servo_id': 8}
      ]
    )
  ])
