from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_settings_file',
            default_value=LaunchConfiguration('sim_settings_file', default='/home/dlee/ros2_ws/src/sar_simulation/sar_config/Sim_Settings.yaml'),
            description='Full path to Sim_Settings parameter file to load'
        ),
        DeclareLaunchArgument(
            'model_types_file',
            default_value=LaunchConfiguration('model_types_file', default='/home/dlee/ros2_ws/src/sar_simulation/sar_config/Model Types.yaml'),
            description='Full path to Model Types parameter file to load'
        ),
        Node(
            package='sar_control',
            executable='practice',
            name='sim_settings_node',
            parameters=[LaunchConfiguration('sim_settings_file'), LaunchConfiguration('model_types_file')]
        )
    ])