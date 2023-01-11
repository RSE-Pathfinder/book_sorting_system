import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #World Parameters
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    world_file_name = 'Simple World'
    urdf_model_name = 'scout_v2.urdf'
    world_path = os.path.join(
        get_package_share_directory('mobile_shelf'),
        'worlds',
        world_file_name
    )
    default_urdf_model_path = os.path.join(
        get_package_share_directory('scout_description'),
        'urdf',
        urdf_model_name
    )

    #Configurations
    headless = LaunchConfiguration('headless')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    urdf_model = LaunchConfiguration('urdf_model')

    #Declarations
    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient'
    )
    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator'
    )
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load'
    )
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file'
    )

    #Robot Parameters
    robot_name_in_model = 'scout_v2'

    #Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.00'

    #Start Gazebo server
    start_gz_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items()
    )

    #Start Gazebo Client
    start_gz_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
    )

    #Launch Robot Node
    entity_spawn_cmd = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-topic', 'robot_description',
            '-x', spawn_x_val,
            '-y', spawn_y_val,
            '-z', spawn_z_val,
            '-Y', spawn_yaw_val],
            output='screen'
        )
    
    #Update Robot Model
    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_model]),
            'use_sim_time': use_sim_time
        }], remappings=remappings
    )

    ld = LaunchDescription()
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(start_gz_server_cmd)
    ld.add_action(start_gz_client_cmd)
    ld.add_action(entity_spawn_cmd)

    return ld