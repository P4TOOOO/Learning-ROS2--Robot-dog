from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Nome do pacote
    package_name = 'robotic_dog'
    share_dir = get_package_share_directory(package_name)

    # Caminho do URDF (não precisa de xacro)
    urdf_file = os.path.join(share_dir, 'urdf', 'robot.urdf')

    # Lê o arquivo URDF diretamente (sem processar xacro)
    with open(urdf_file, 'r') as infp:
        robot_urdf = infp.read()

    # Caminho do config do RViz (crie esse arquivo salvando do RViz se não existir)
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    # Argumento para ativar/desativar a GUI
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )

    show_gui = LaunchConfiguration('gui')

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
