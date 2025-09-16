from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dev_arg = DeclareLaunchArgument("dev", default_value="/dev/ttyACM0",
                                    description="Serial device path")
    baud_arg = DeclareLaunchArgument("baud", default_value="115200",
                                     description="Baud rate")
    buf_arg = DeclareLaunchArgument("read_buffer_bytes", default_value="2048",
                                    description="Read buffer size (bytes)")
    ns_arg = DeclareLaunchArgument("namespace", default_value="",
                                   description="ROS namespace")
    name_arg = DeclareLaunchArgument("node_name", default_value="rosserial_agent",
                                     description="Node name")
    params_arg = DeclareLaunchArgument("params_file", default_value="",
                                       description="YAML file with ROS 2 parameters")

    dev = LaunchConfiguration("dev")
    baud = LaunchConfiguration("baud")
    rbuf = LaunchConfiguration("read_buffer_bytes")
    ns = LaunchConfiguration("namespace")
    nname = LaunchConfiguration("node_name")
    pfile = LaunchConfiguration("params_file")

    agent = Node(
        package="rosserial_agent_ros2",
        executable="rosserial_agent",
        namespace=ns,
        name=nname,
        parameters=[{
            "dev": dev,
            "baud": baud,
            "read_buffer_bytes": rbuf,
        }, pfile],
        output="screen",
    )

    return LaunchDescription([
        dev_arg, baud_arg, buf_arg, ns_arg, name_arg, params_arg,
        agent
    ])
