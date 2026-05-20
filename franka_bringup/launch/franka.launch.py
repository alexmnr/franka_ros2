import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)

    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'urdf',
                                     'panda_arm.urdf.xacro')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file,
         ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands])

    franka_controllers = PathJoinSubstitution(
        [
            FindPackageShare('franka_bringup'),
            'config',
            'ros2_controllers.yaml',
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            description='Hostname or IP address of the robot.'),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description="Fake sensor commands. Only valid when '{}' is true".format(
                use_fake_hardware_parameter_name)),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{'robot_description': robot_description}, franka_controllers],
            arguments=["--ros-args", "--log-level", "info"],
            output="screen",
            ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_robot_state_broadcaster'],
            output='screen',
            condition=UnlessCondition(use_fake_hardware),
        ),
    ])
