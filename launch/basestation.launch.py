from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)

    nodes = []

    if mode == "dev":
        # Development mode: backend + vite dev server, no browser (open manually)
        nodes.append(Node(package="mrover", executable="gui_backend.sh", name="teleop_backend"))
        nodes.append(Node(package="mrover", executable="gui_frontend.sh", name="teleop_frontend", output="screen"))
    elif mode == "prod":
        # Production mode: backend serves built static files + chromium
        nodes.append(Node(package="mrover", executable="gui_prod.sh", name="teleop_prod"))
        nodes.append(Node(package="mrover", executable="gui_chromium.sh", name="teleop_chromium"))
    else:
        raise ValueError(f"Invalid mode: {mode}. Must be 'dev' or 'prod'")

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="dev",
                description="Launch mode: dev (vite dev server, open browser manually) or prod (optimized build + chromium)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
