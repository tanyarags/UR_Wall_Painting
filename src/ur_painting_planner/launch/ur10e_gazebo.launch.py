from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

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
            "name:=",
            "ur",
            " ",
            "ur_type:=ur10e",            
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
    
    # Load RViz config
    rviz_config_file = os.path.join(curr_pkg_dir, "rviz", "view_robot.rviz")
    
    # Create nodes and launch files to include
    
    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    # 2. Gazebo launch
    # Start Gazebo with world
    start_gazebo_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', LaunchConfiguration("world_path")],
        output='screen',
    )

    start_gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # 3. Spawn robot in Gazebo
    # Define robot starting pose
    robot_pose = ["-x", "-0.5", "-y", "0", "-z", "0.1", "-R", "0", "-P", "0", "-Y", "0"]

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'ur', '-topic', 'robot_description'] + robot_pose,
        output='screen'
    )

    # 4. RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # 5. Include MoveIt launch
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), 'launch', 'ur_moveit.launch.py'])
        ),
        launch_arguments={
            'name' : 'ur',
            'ur_type': ur_type,
            'robot_description': robot_description_content,
            'use_sim_time': 'true'
        }.items()
    )

    # 6. Controllers
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(curr_pkg_dir, "config", "controller.yaml"),
        ],
    )

    # Delay loading controllers until robot is spawned
    load_joint_state_controller = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )

    load_joint_trajectory_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )

    return LaunchDescription(
        declared_arguments + 
        [
            start_gazebo_server,
            start_gazebo_client,
            robot_state_publisher,
            spawn_robot,
            controller_manager,
            load_joint_state_controller,
            load_joint_trajectory_controller,
            #rviz_node,
            moveit_launch,
            
        ]
    )