from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(file_path):
    # package_path = get_package_share_directory(package_name)
    # absolute_file_path = os.path.join(package_path, file_path)
    absolute_file_path = file_path
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        raise RuntimeError(f"Failed to load yaml file {absolute_file_path}: {str(e)}")

def generate_launch_description():
    
    # Get package directories
    ur_description_dir = get_package_share_directory('ur_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    curr_pkg_dir = get_package_share_directory('ur_painting_planner')

    # Declare arguments
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
           DeclareLaunchArgument(
            "world_path",
            default_value=os.path.join(
                FindPackageShare("ur_painting_planner").find("ur_painting_planner"),
                "worlds",
                "painting_world.world"
            ),
            description="Path to the world file to load"
        )
    )

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")

    #TODO: remove unnecessary arguments
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "sim_gazebo:=true",
            " ",
            "simulation_controllers:=",
            os.path.join(curr_pkg_dir, "config", "controller.yaml"),
        ])
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}


   # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]
            ),
            " ",
            "name:=ur",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            " ",
            "prefix:=",
            tf_prefix,
            " ",
        ]
    )

    # Load SRDF
    robot_description_semantic_content = """<?xml version="1.0" encoding="UTF-8"?>
    <robot name="ur">
        <group name="ur_manipulator">
            <chain base_link="base_link" tip_link="tool0"/>
        </group>

        <!-- Add this virtual joint specification -->
        <virtual_joint name="world_joint" type="fixed" parent_frame="world" child_link="base_link"/>
        
        <group_state name="home" group="ur_manipulator">
            <joint name="shoulder_pan_joint" value="0"/>
            <joint name="shoulder_lift_joint" value="-1.5707"/>
            <joint name="elbow_joint" value="0"/>
            <joint name="wrist_1_joint" value="-1.5707"/>
            <joint name="wrist_2_joint" value="0"/>
            <joint name="wrist_3_joint" value="0"/>
        </group_state>

        <disable_collisions link1="base_link" link2="shoulder_link" reason="Adjacent"/>
        <!-- Add other necessary collision entries -->
    </robot>"""

    robot_description_semantic = {
    "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }
    # robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    # Load Kinematics
    file_path = os.path.join(get_package_share_directory('ur_painting_planner'), "config", "kinematics.yaml")
    kinematics_yaml = load_yaml(file_path)

    planning_scene_monitor_parameters = {
    "publish_planning_scene": True,
    "publish_geometry_updates": True,
    "publish_state_updates": True,
    "publish_transforms_updates": True,
}

    # Add your motion planner node
    ur_motion_planner_node = Node(
        package="ur_painting_planner",
        executable="ur_motion_planner_node",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            planning_scene_monitor_parameters
        ],
        output="screen"
    )

    
    
    return LaunchDescription(
        declared_arguments +          
        [
            ur_motion_planner_node,
            
        ]
    )