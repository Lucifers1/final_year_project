import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # 1. Package Paths
    pkg_path = get_package_share_directory('portable_laser_scanner')
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')

    # 2. URDF File
    urdf_path = os.path.join(pkg_path, 'urdf', 'portable_laser_scanner.urdf.xacro')
    
    world_path = os.path.join(pkg_path, 'worlds', 'gz_world.sdf')
    
    # Process the URDF file
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # 3. Launch Gazebo Harmonic (gz_sim)
    # We use an empty world by default. 
    # '-r' runs the simulation immediately on startup.
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -s {world_path}'}.items(),
    )

    # 4. Spawn the Robot
    # In Harmonic, we use the 'create' node from ros_gz_sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'motor_test_robot',
            '-z', '0.0', # Spawn slightly above ground
            '-y', '0.0',
            '-x', '0.0'
        ],
        output='screen'
    )

    # 5. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # 6. Joint State Publisher GUI (Optional, for RViz visualization of joints)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # 7. ROS-Gazebo Bridge
    # This bridges the Lidar data from Gazebo (GZ topic) to ROS (ROS topic)
    # Mapping: /scan (GZ) -> /scan (ROS)
    # Mapping: /clock (GZ) -> /clock (ROS)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/motor_test_robot/joint/bar_to_semi_octagon/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/motor_test_robot/joint/base_to_bar/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
        ],
        output='screen'
    )

    # 8. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        # arguments=['-d', os.path.join(pkg_path, 'rviz', 'config.rviz')]
    )

    return LaunchDescription([
        gazebo_sim,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        bridge,
        rviz_node
    ])
