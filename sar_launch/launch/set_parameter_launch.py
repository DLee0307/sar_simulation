from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    sim_settings_file = LaunchConfiguration('sim_settings_file', default='/home/dlee/ros2_ws/src/sar_simulation/sar_config/Sim_Settings.yaml')
    model_types_file = LaunchConfiguration('model_types_file', default='/home/dlee/ros2_ws/src/sar_simulation/sar_config/Model_Types.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_settings_file',
            default_value=sim_settings_file,
            description='Full path to Sim_Settings parameter file to load'
        ),
        DeclareLaunchArgument(
            'model_types_file',
            default_value=model_types_file,
            description='Full path to Model Types parameter file to load'
        ),
        Node(
            package='sar_control',
            executable='SAR_Controller',
            name='SAR_Controller',
            parameters=[sim_settings_file, model_types_file]
        )
    ])