from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arduinobot_description"),
                "launch",
                "gazebo.launch.xml"
            )
        )
    )

    controller = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arduinobot_controller"),
                "launch",
                "controller.launch.xml"
            )
        )
    )

    moveit = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arduinobot_moveit"),
                "launch",
                "moveit.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    remote_interface = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arduinobot_remote"),
                "launch",
                "remote_interface.launch.py"
            )
        )
    )

    return LaunchDescription([
        gazebo,
        controller,
        moveit,
        remote_interface
    ])