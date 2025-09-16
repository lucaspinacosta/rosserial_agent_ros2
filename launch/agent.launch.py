from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Args
    dev_arg = DeclareLaunchArgument("dev", default_value="/dev/ttyACM0")
    baud_arg = DeclareLaunchArgument("baud", default_value="115200")
    buf_arg = DeclareLaunchArgument("read_buffer_bytes", default_value="2048")
    ns_arg = DeclareLaunchArgument("namespace", default_value="")
    name_arg = DeclareLaunchArgument(
        "node_name", default_value="rosserial_agent")
    params_arg = DeclareLaunchArgument("params_file", default_value="")

    def launch_setup(context, *args, **kwargs):
        # Build parameter list; only append params_file if provided (non-empty)
        params = [{
            "dev": LaunchConfiguration("dev"),
            "baud": LaunchConfiguration("baud"),
            "read_buffer_bytes": LaunchConfiguration("read_buffer_bytes"),
        }]
        pfile = LaunchConfiguration("params_file").perform(context)
        if pfile:
            params.append(pfile)

        node = Node(
            package="rosserial_agent_ros2",
            executable="rosserial_agent",
            namespace=LaunchConfiguration("namespace"),
            name=LaunchConfiguration("node_name"),
            parameters=params,
            output="screen",
        )
        return [node]

    return LaunchDescription([
        dev_arg, baud_arg, buf_arg, ns_arg, name_arg, params_arg,
        OpaqueFunction(function=launch_setup),
    ])
