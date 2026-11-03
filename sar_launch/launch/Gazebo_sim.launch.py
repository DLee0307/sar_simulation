import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    yaml_file_path = os.path.expanduser('~/ros2_ws/src/sar_simulation/sar_config/Sim_Settings.yaml')

    with open(yaml_file_path, 'r') as file:
        config = yaml.safe_load(file)

    SAR_TYPE = config['SAR_Controller']['ros__parameters']['SAR_SETTINGS']['SAR_Type']
    SAR_CONFIG = config['SAR_Controller']['ros__parameters']['SAR_SETTINGS']['SAR_Config']
    SAR_SDF_Path = os.path.expanduser(
        f'~/ros2_ws/src/sar_simulation/sar_gazebo/models/{SAR_TYPE}/Configs/{SAR_CONFIG}/model.sdf'
    )

    PLANE_TYPE = config['SAR_Controller']['ros__parameters']['PLANE_SETTINGS']['Plane_Type']
    PLANE_CONFIG = config['SAR_Controller']['ros__parameters']['PLANE_SETTINGS']['Plane_Config']
    Plane_SDF_Path = os.path.expanduser(
        f'~/ros2_ws/src/sar_simulation/sar_gazebo/models/{PLANE_TYPE}/Configs/{PLANE_CONFIG}/model.sdf'
    )

    GROUND_TYPE = config['SAR_Controller']['ros__parameters']['GROUND_SETTINGS']['Ground_Type']
    GROUND_CONFIG = config['SAR_Controller']['ros__parameters']['GROUND_SETTINGS']['Ground_Config']
    Ground_SDF_Path = os.path.expanduser(
        f'~/ros2_ws/src/sar_simulation/sar_gazebo/models/{GROUND_TYPE}/Configs/{GROUND_CONFIG}/model.sdf'
    )

    models_root = os.path.expanduser('~/ros2_ws/src/sar_simulation/sar_gazebo/models')
    world_path = os.path.expanduser('~/ros2_ws/src/sar_simulation/sar_gazebo/worlds/empty.world')

    # include로 참조되는 base 모델 경로들
    SAR_BASE_PATH = os.path.join(models_root, 'SAR_Base')
    PLANE_BASE_PATH = os.path.join(models_root, 'Plane_Base_Model')
    GROUND_BASE_PATH = os.path.join(models_root, 'Ground_Base_Model')

    # config 모델 타입 경로들
    SAR_TYPE_PATH = os.path.join(models_root, SAR_TYPE)
    PLANE_TYPE_PATH = os.path.join(models_root, PLANE_TYPE)
    GROUND_TYPE_PATH = os.path.join(models_root, GROUND_TYPE)

    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    gz_resource_paths = [
        models_root,
        SAR_BASE_PATH,
        PLANE_BASE_PATH,
        GROUND_BASE_PATH,
        SAR_TYPE_PATH,
        PLANE_TYPE_PATH,
        GROUND_TYPE_PATH,
    ]

    if existing_gz_path:
        gz_resource_paths.append(existing_gz_path)

    combined_gz_resource_path = ':'.join(gz_resource_paths)

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    CLOCK_BRIDGE = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    SAR_SPAWN = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', SAR_SDF_Path, '-z', '0.4'],
        output='screen',
    )

    PLANE_SPAWN = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', Plane_SDF_Path, '-x', '5.0', '-z', '2.75'], #2.85
        output='screen',
    )

    SAR_CONTROLLER = Node(
        package='sar_control',
        executable='SAR_Controller',
        name='SAR_Controller',
        output='screen',
        parameters=[
            yaml_file_path,
            {'use_sim_time': True}
        ],
    )

    GROUND_SPAWN = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', Ground_SDF_Path, '-x', '0.0', '-z', '-10.0'],
        output='screen',
    )

    # controller_terminal_command = ExecuteProcess(
    #     cmd=[
    #         'gnome-terminal',
    #         '--disable-factory',
    #         '--geometry', '85x46+1050+0',
    #         '--',
    #         'bash', '-lc',
    #         'ros2 run sar_control SAR_Controller --ros-args -p use_sim_time:=true'
    #     ],
    #     output='screen'
    # )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=combined_gz_resource_path
        ),

        gazebo,
        TimerAction(period=1.0, actions=[CLOCK_BRIDGE]),
        TimerAction(period=1.5, actions=[GROUND_SPAWN]),
        TimerAction(period=2.0, actions=[PLANE_SPAWN]),
        TimerAction(period=2.5, actions=[SAR_SPAWN]),
        TimerAction(period=3.0, actions=[SAR_CONTROLLER]),
    ])