from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from core_common.config import SCENARIOS


def generate_launch_description():
    config = Path(get_package_share_directory('core_bringup')) / 'config'
    mode = LaunchConfiguration('mode')
    scenario = LaunchConfiguration('scenario')
    base = {'mode': mode, 'data_dir': ParameterValue(LaunchConfiguration('data_dir'), value_type=str)}
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='fake', choices=['fake']),
        DeclareLaunchArgument('scenario', default_value='missing', choices=list(SCENARIOS)),
        DeclareLaunchArgument('data_dir', default_value='data'),
        DeclareLaunchArgument('voice', default_value='false'),
        DeclareLaunchArgument('llm', default_value='false'),
        Node(package='core_robot', executable='robot_node', name='robot_node', output='screen',
             parameters=[str(config / 'robot.yaml'), base, {'scenario': scenario}]),
        Node(package='core_vision', executable='vision_node', name='vision_node', output='screen',
             parameters=[{'mode': mode, 'scenario': scenario}]),
        Node(package='core_control', executable='control', name='control', output='screen',
             parameters=[str(config / 'workspace.yaml'), base]),
        Node(package='core_language', executable='voice_node', output='screen',
             condition=IfCondition(LaunchConfiguration('voice'))),
        Node(package='core_language', executable='llm_node', output='screen',
             condition=IfCondition(LaunchConfiguration('llm'))),
    ])
