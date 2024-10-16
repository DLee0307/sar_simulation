import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Define the path to the YAML file
    yaml_file_path = os.path.expanduser('~/ros2_ws/src/sar_simulation/sar_config/Sim_Settings.yaml')

    # Load the YAML file
    with open(yaml_file_path, 'r') as file:
        config = yaml.safe_load(file)

    # Extract the SAR_Type1
    SAR_TYPE = config['SAR_Controller']['ros__parameters']['SAR_SETTINGS']['SAR_Type']
    SAR_CONFIG = config['SAR_Controller']['ros__parameters']['SAR_SETTINGS']['SAR_Config']
    SAR_SDF_Path = os.path.expanduser(f'~/ros2_ws/src/sar_simulation/sar_gazebo/models/{SAR_TYPE}/Configs/{SAR_CONFIG}/model.sdf')

    PLANE_TYPE = config['SAR_Controller']['ros__parameters']['PLANE_SETTINGS']['Plane_Type']
    PLANE_CONFIG = config['SAR_Controller']['ros__parameters']['PLANE_SETTINGS']['Plane_Config']
    Plane_SDF_Path = os.path.expanduser(f'~/ros2_ws/src/sar_simulation/sar_gazebo/models/{PLANE_TYPE}/Configs/{PLANE_CONFIG}/model.sdf')

    GROUND_TYPE = config['SAR_Controller']['ros__parameters']['GROUND_SETTINGS']['Ground_Type']
    GROUND_CONFIG = config['SAR_Controller']['ros__parameters']['GROUND_SETTINGS']['Ground_Config']
    Ground_SDF_Path = os.path.expanduser(f'~/ros2_ws/src/sar_simulation/sar_gazebo/models/{GROUND_TYPE}/Configs/{GROUND_CONFIG}/model.sdf')

    # SET GZ_SIM_RESOURCE_PATH
    SAR_SDF_Path_BASE = os.path.expanduser(f'~/ros2_ws/src/sar_simulation/sar_gazebo/models/{SAR_TYPE}')
    Plane_SDF_Path_BASE = os.path.expanduser(f'~/ros2_ws/src/sar_simulation/sar_gazebo/models/{PLANE_TYPE}')
    Ground_SDF_Path_BASE = os.path.expanduser(f'~/ros2_ws/src/sar_simulation/sar_gazebo/models/{GROUND_TYPE}')

    COMBINED_PATH_BASE =  f"{SAR_SDF_Path_BASE}:{Plane_SDF_Path_BASE}:{Ground_SDF_Path_BASE}"

    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Gazebo Sim
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    #     ),
    #     launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    # )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {os.path.expanduser("~/ros2_ws/src/sar_simulation/sar_gazebo/worlds/empty.world")}'}.items(),  # 절대 경로로 empty.world 파일 설정
    )

    # SAR Spawn
    SAR_SPAWN = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', SAR_SDF_Path, '-z', '0.4'],
        output='screen',
    )

    # Plane Spawn
    PLANE_SPAWN = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', Plane_SDF_Path, '-x', '3.0', '-z', '2.50'], # 2.55 # 2.59(last)
        output='screen',
    )

    # Ground Spawn
    GROUND_SPAWN = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', Ground_SDF_Path, '-z', '-10.0'],
        output='screen',
    )    

    # Clock Bridge
    CLOCK_BRIDGE = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    SAR_CONTROLLER = Node(
        package='sar_control',
        executable='SAR_Controller',
        name='sar_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],  # use_sim_time을 직접 노드의 파라미터로 설정
    )

    controller_terminal_command = ExecuteProcess(
        cmd=['gnome-terminal', '--disable-factory', '--geometry', '85x46+1050+0', '--',
             'ros2', 'run', 'sar_control', 'SAR_Controller', '--ros-args', '--param', 'use_sim_time:=true'],
        shell=True
    )
    
    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value= COMBINED_PATH_BASE
        ),     
        gazebo,
        SAR_SPAWN,
        PLANE_SPAWN,
        GROUND_SPAWN,
        CLOCK_BRIDGE,
        SAR_CONTROLLER
    ])


