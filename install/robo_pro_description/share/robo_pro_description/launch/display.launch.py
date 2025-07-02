from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_path
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess


def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('robo_pro_description'), 'urdf', 'robo.urdf.xacro')

    rviz_config_path = os.path.join(get_package_share_path('robo_pro_description'), 'config', 'urdf_config.rviz')

        # LiDAR launch file path from ldlidar_stl_ros2 package
    lidar_launch_file = PathJoinSubstitution([FindPackageShare('ldlidar_stl_ros2'), 'launch', 'ld06.launch.py'])

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    lidar_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(lidar_launch_file)
    )

    micro_ros_agent_node = ExecuteProcess(
    cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'udp4', '--port', '8888'],
    output='screen'
    )

    odomerty = Node(
        package='control',
        executable='odom_node',  # Replace with your actual executable name
        name='odom_node',
        output='screen'

    )

    tf_base_footprint = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0.0325', '0', '0', '0', 'base_footprint', 'base_link'],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    slam_toolbox_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(get_package_share_path('slam_toolbox'), 'launch', 'online_async_launch.py')
    ]),
    launch_arguments={
        'params_file': '/home/essam/mr_ws/src/robo_pro_description/config/mapper_params_online_async.yaml'
    }.items()
)



    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_path('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
        micro_ros_agent_node,
        odomerty,
        lidar_node,
        slam_toolbox_launch,
        tf_base_footprint,
        #gazebo,
        #spawn_entity,
    ])