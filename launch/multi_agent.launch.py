from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_agent(namespace, node_name, dev, baud, rbuf, pfile):
    return Node(
        package="rosserial_agent_ros2",
        executable="rosserial_agent",
        namespace=namespace,
        name=node_name,
        parameters=[{
            "dev": dev,
            "baud": int(baud),
            "read_buffer_bytes": int(rbuf),
        }, pfile],
        output="screen",
    )


def generate_launch_description():
    dev1_arg = DeclareLaunchArgument("dev1", default_value="/dev/ttyACM0")
    dev2_arg = DeclareLaunchArgument("dev2", default_value="/dev/ttyACM1")
    baud_arg = DeclareLaunchArgument("baud", default_value="115200")
    buf_arg = DeclareLaunchArgument("read_buffer_bytes", default_value="2048")
    params_arg = DeclareLaunchArgument("params_file", default_value="",
                                       description="YAML params file")

    dev1 = LaunchConfiguration("dev1")
    dev2 = LaunchConfiguration("dev2")
    baud = LaunchConfiguration("baud")
    rbuf = LaunchConfiguration("read_buffer_bytes")
    pfile = LaunchConfiguration("params_file")

    agent1 = make_agent("board1", "rosserial_agent_1", dev1, baud, rbuf, pfile)
    agent2 = make_agent("board2", "rosserial_agent_2", dev2, baud, rbuf, pfile)

    return LaunchDescription([dev1_arg, dev2_arg, baud_arg, buf_arg, params_arg,
                              agent1, agent2])
