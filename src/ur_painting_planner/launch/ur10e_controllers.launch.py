# ur10e_controllers.launch.py
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('ur_painting_planner')
    
    # Load controllers configuration file
    controller_config = os.path.join(
        pkg_dir,
        'config',
        'controllers.yaml'
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delay starting the trajectory controller until joint state broadcaster is ready
    delay_trajectory_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
                )
            ],
        )
    )

    return LaunchDescription([
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_trajectory_controller_spawner,
    ])