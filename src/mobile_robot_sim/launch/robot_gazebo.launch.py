from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = FindPackageShare(package='mobile_robot_sim').find('mobile_robot_sim')
    default_model_path = os.path.join(pkg_share, 'urdf', 'DeliveryRobot_2.urdf')
    world_file_path = os.path.join(pkg_share, 'world', 'my_world.sdf')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf_config.yaml')

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': ParameterValue(Command(['xacro ', default_model_path]), value_type=str),
                'use_sim_time': True
            }
        ]
    )

    # Gazebo Sim Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file_path}'}.items()
    )

    # ROS-Gazebo Bridge
    # /cmd_vel on ROS side -> /model/DeliveryRobot_Sim/cmd_vel on Gazebo side (MecanumDrive listens here)
    # /model/DeliveryRobot_Sim/odometry on Gazebo side -> /odom on ROS side
    bridge_gz = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/DeliveryRobot_Sim/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/DeliveryRobot_Sim/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/ultrasonic/front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ultrasonic/back@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ultrasonic/left@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ultrasonic/right@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lid_position@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            ('/model/DeliveryRobot_Sim/cmd_vel', '/cmd_vel'),
            ('/model/DeliveryRobot_Sim/odometry', '/odom'),
        ]
    )

    # Spawn robot
    node_gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'DeliveryRobot_Sim',
            '-x', '0.0', '-y', '0.0', '-z', '0.3',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'
        ],
        parameters=[{'use_sim_time': True}]
    )

    # EKF Localization Node
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}]
    )

    # Lid Control Node (Gazebo-native JointPositionController handles the physics)
    lid_control_node = Node(
        package='mobile_robot_sim',
        executable='lid_control_node.py',
        name='lid_control_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    delay_lid_control = TimerAction(period=5.0, actions=[lid_control_node])

    # Start/Stop Node (gates /cmd_vel_input -> /cmd_vel)
    start_stop_node = Node(
        package='mobile_robot_sim',
        executable='start_stop_node.py',
        name='start_stop_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    delay_start_stop = TimerAction(period=5.0, actions=[start_stop_node])

    return LaunchDescription([
        gazebo,
        bridge_gz,
        node_gz_spawn_entity,
        robot_state_publisher_node,
        robot_localization_node,
        delay_lid_control,
        delay_start_stop,
    ])