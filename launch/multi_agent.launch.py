from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Args
    dev1_arg = DeclareLaunchArgument("dev1", default_value="/dev/ttyACM0")
    dev2_arg = DeclareLaunchArgument("dev2", default_value="/dev/ttyACM1")
    baud_arg = DeclareLaunchArgument("baud", default_value="115200")
    buf_arg = DeclareLaunchArgument("read_buffer_bytes", default_value="2048")
    params_arg = DeclareLaunchArgument("params_file", default_value="")

    def launch_setup(context, *args, **kwargs):
        # Common params (as LaunchConfiguration objects; do NOT cast to int())
        common = {
            "baud": LaunchConfiguration("baud"),
            "read_buffer_bytes": LaunchConfiguration("read_buffer_bytes"),
        }
        def params_list(dev): return [{"dev": dev, **common}]
        pfile = LaunchConfiguration("params_file").perform(context)
        if pfile:
            # Append YAML if provided
            def with_yaml(pl):
                out = list(pl)
                out.append(pfile)
                return out
        else:
            def with_yaml(pl): return pl  # no-op

        agent1 = Node(
            package="rosserial_agent_ros2",
            executable="rosserial_agent",
            namespace="board1",
            name="rosserial_agent_1",
            parameters=with_yaml(params_list(LaunchConfiguration("dev1"))),
            output="screen",
        )
        agent2 = Node(
            package="rosserial_agent_ros2",
            executable="rosserial_agent",
            namespace="board2",
            name="rosserial_agent_2",
            parameters=with_yaml(params_list(LaunchConfiguration("dev2"))),
            output="screen",
        )
        return [agent1, agent2]

    return LaunchDescription([
        dev1_arg, dev2_arg, baud_arg, buf_arg, params_arg,
        OpaqueFunction(function=launch_setup),
    ])
