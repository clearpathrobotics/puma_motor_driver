from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  # Launch Configurations
  parameters = LaunchConfiguration('parameters')

  # Launch Arguments
  arg_parameters = DeclareLaunchArgument(
    'parameters',
    default_value=PathJoinSubstitution([
      FindPackageShare('puma_motor_driver'),
      'config',
      'dingo_d.yaml'
    ]))

  # Puma Node
  puma_node = Node(
    package='puma_motor_driver',
    executable='multi_puma_node',
    parameters=[parameters],
    output='screen'
  )

  ld = LaunchDescription()
  ld.add_action(arg_parameters)
  ld.add_action(puma_node)
  return ld
