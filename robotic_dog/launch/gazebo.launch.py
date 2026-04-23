from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('robotic_dog')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf')

    return LaunchDescription([

        SetParameter(name='use_sim_time', value=True),
        # 1. Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-r empty.sdf'}.items()
        ),

        # 2. Bridge (clock + tf + odom)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        TimerAction(
            period=4.0,   
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    parameters=[
                        {'robot_description': open(urdf_path, 'r').read()},
                        {'use_sim_time': True}
                    ],
                    output='screen',
                ),

                # Spawn the robot
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-topic', '/robot_description',
                        '-name', 'robotic_dog',
                        '-allow_renaming', 'false',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '1.5',
                        '-Y', '0.0'
                    ],
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                ),

                # Controller Manager
                Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    parameters=[
                        os.path.join(pkg_dir, 'config', 'controllers.yaml'),
                        {'use_sim_time': True}
                    ],
                    output='screen',
                ),
                TimerAction(
                period=6.0,
                actions=[
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                        output='screen',
                        parameters=[{'use_sim_time': True}]
                    ),
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['position_controllers', '--controller-manager', '/controller_manager'],
                        output='screen',
                        parameters=[{'use_sim_time': True}]
                    ),
                    Node(
                        package='robotic_dog',
                        executable='robotic_dog_node',
                        name='robotic_dog_node',
                        parameters=[{'use_sim_time': True}],
                        output='screen',
                    ),
                ]
            ),
            ]
        ),
    ])
