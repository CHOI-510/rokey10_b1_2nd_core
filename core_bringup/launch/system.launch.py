from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from core_common import topics
from core_common.config import SCENARIOS


def generate_launch_description():
    config = Path(get_package_share_directory('core_bringup')) / 'config'
    mode = LaunchConfiguration('mode')
    fake = IfCondition(PythonExpression(["'", mode, "' == 'fake'"]))
    real = IfCondition(PythonExpression(["'", mode, "' == 'real'"]))
    camera = LaunchConfiguration('camera_topic')
    scenario = LaunchConfiguration('scenario')
    base = {'mode': mode, 'data_dir': ParameterValue(LaunchConfiguration('data_dir'), value_type=str)}
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='fake', choices=['fake', 'inspection', 'real']),
        DeclareLaunchArgument('scenario', default_value='missing', choices=list(SCENARIOS)),
        DeclareLaunchArgument('camera_topic', default_value=topics.CAMERA_COLOR,
                              description='Raw wrist Image for real cycle and default Vision 1 input.'),
        DeclareLaunchArgument('vision_camera_topic', default_value=camera,
                              description='Raw image showing both bolts at the calibrated inspection pose.'),
        DeclareLaunchArgument('vision_roi_file', default_value='~/.config/core_vision/vision1.json'),
        DeclareLaunchArgument('dsr_stop_service', default_value=topics.DSR_MOVE_STOP),
        DeclareLaunchArgument('data_dir', default_value='data'),
        DeclareLaunchArgument('voice', default_value='false'),
        DeclareLaunchArgument('llm', default_value='false'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('preview_enabled', default_value=PythonExpression([
            "'false' if '", mode, "' == 'fake' else 'true'"])),
        DeclareLaunchArgument('wrist_image_type', default_value='raw', choices=['raw', 'compressed']),
        DeclareLaunchArgument('side_image_type', default_value='raw', choices=['raw', 'compressed']),
        DeclareLaunchArgument('wrist_image_topic', default_value=camera),
        DeclareLaunchArgument('side_image_topic', default_value=''),
        DeclareLaunchArgument('wrench_topic', default_value=''),
        Node(package='core_robot', executable='robot_node', name='robot_node', output='screen',
             condition=fake, parameters=[str(config / 'robot.yaml'), base, {
                 'scenario': scenario}]),
        Node(package='core_vision', executable='vision_node', name='vision_node', output='screen',
             condition=fake, parameters=[{'mode': mode, 'scenario': scenario}]),
        Node(package='core_robot', executable='robot_node', name='robot_node', output='screen',
             condition=real, parameters=[str(config / 'robot.yaml'), base, {
                 'camera_topic': ParameterValue(camera, value_type=str),
                 'dsr_stop_service': ParameterValue(LaunchConfiguration('dsr_stop_service'), value_type=str),
             }]),
        Node(package='core_vision', executable='vision1_node', name='vision1_node', output='screen',
             condition=real, parameters=[{
                 'real_camera': True,
                 'camera_topic': ParameterValue(LaunchConfiguration('vision_camera_topic'), value_type=str),
                 'roi_file': ParameterValue(LaunchConfiguration('vision_roi_file'), value_type=str),
             }]),
        Node(package='core_control', executable='control', name='control', output='screen',
             parameters=[str(config / 'workspace.yaml'), base]),
        Node(package='core_control', executable='monitor_gui', name='monitor_gui', output='screen',
             condition=IfCondition(LaunchConfiguration('gui')),
             parameters=[{'mode': mode, 'preview_enabled': ParameterValue(
                 LaunchConfiguration('preview_enabled'), value_type=bool), **{
                 name: ParameterValue(LaunchConfiguration(name), value_type=str)
                 for name in ('wrist_image_topic', 'side_image_topic', 'wrist_image_type',
                              'side_image_type', 'wrench_topic')
             }}]),
        Node(package='core_language', executable='voice_node', output='screen',
             condition=IfCondition(LaunchConfiguration('voice'))),
        Node(package='core_language', executable='llm_node', output='screen',
             condition=IfCondition(LaunchConfiguration('llm'))),
    ])
