#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_ros_planning_interface.moveit_move_group_interface import MoveGroupInterface
from moveit_msgs.msg import Constraints, OrientationConstraint
from geometry_msgs.msg import Pose, PoseArray, Vector3, Point, Quaternion
import numpy as np
from dataclasses import dataclass

from rclpy.action import ActionClient
from control_msgs.action import FollowCartesianTrajectory
from std_msgs.msg import Bool
from rclpy.publisher import Publisher

@dataclass
class WallParameters:
    width: float
    height: float
    wall_pose: Pose
    window_pose: Pose
    window_dimensions: Vector3

@dataclass
class PaintingParameters:
    spray_width: float
    overlap: float
    standoff: float
    speed: float

class ForceController:
    def __init__(self):
        self.client = ActionClient(node=self, 
                                 action_type=FollowCartesianTrajectory,
                                 action_name='force_control')
    
    def activate(self):
        # Configure force control parameters
        params = {'force_threshold': 10.0, 'stiffness': 1000.0}
        self.client.send_goal(params)
    
    def deactivate(self):
        self.client.cancel_goal()

class SprayController:
    def __init__(self):
        self.spray_pub = Publisher('/spray_control', Bool)
    
    def enable(self):
        self.spray_pub.publish(Bool(data=True))
    
    def disable(self):
        self.spray_pub.publish(Bool(data=False))

class WallPaintingPlanner(Node):
    def __init__(self):
        super().__init__('painting_node2')
        self.move_group = MoveGroupInterface(node=self, 
                                           group_name="manipulator",
                                           robot_description="robot_description")
        
        self.setup_parameters()
        self.setup_controllers()

    def setup_parameters(self):
        # Declare individual parameters
        self.declare_parameter('wall.width', 3.0)
        self.declare_parameter('wall.height', 2.4)
        self.declare_parameter('window.position.x', 1.2)
        self.declare_parameter('window.position.y', 1.5)
        self.declare_parameter('window.dimensions.width', 0.8)
        self.declare_parameter('window.dimensions.height', 1.0)
        self.declare_parameter('painting.spray_width', 0.15)
        self.declare_parameter('painting.overlap', 0.1)
        self.declare_parameter('painting.standoff', 0.2)
        self.declare_parameter('painting.speed', 0.3)

        # Get parameter values
        self.wall_params = WallParameters(
            width=self.get_parameter('wall.width').value,
            height=self.get_parameter('wall.height').value,
            wall_pose=Pose(),  # Set default or get from TF
            window_pose=Pose(position=Point(
                x=self.get_parameter('window.position.x').value,
                y=self.get_parameter('window.position.y').value,
                z=0.0)),
            window_dimensions=Vector3(
                x=self.get_parameter('window.dimensions.width').value,
                y=self.get_parameter('window.dimensions.height').value,
                z=0.0)
        )

        self.paint_params = PaintingParameters(
            spray_width=self.get_parameter('painting.spray_width').value,
            overlap=self.get_parameter('painting.overlap').value,
            standoff=self.get_parameter('painting.standoff').value,
            speed=self.get_parameter('painting.speed').value
        )

    def setup_controllers(self):
        # Initialize force controller
        self.force_controller = ForceController()  
        
        # Initialize spray controller
        self.spray_controller = SprayController()

        
    def generate_coverage_path(self) -> PoseArray:
        path = PoseArray()
        current_height = 0
        
        # Generate zigzag pattern
        while current_height < self.wall_params.height:
            # Left to right
            for x in np.arange(0, self.wall_params.width, 
                             self.paint_params.spray_width * (1 - self.paint_params.overlap)):
                if not self.is_window_region(x, current_height):
                    pose = self.create_spray_pose(x, current_height)
                    path.poses.append(pose)
            
            current_height += self.paint_params.spray_width
            
            # Right to left
            if current_height < self.wall_params.height:
                for x in np.arange(self.wall_params.width, 0, 
                                 -self.paint_params.spray_width * (1 - self.paint_params.overlap)):
                    if not self.is_window_region(x, current_height):
                        pose = self.create_spray_pose(x, current_height)
                        path.poses.append(pose)
                
                current_height += self.paint_params.spray_width
        
        return path

    def is_window_region(self, x: float, y: float) -> bool:
        window_x = self.wall_params.window_pose.position.x
        window_y = self.wall_params.window_pose.position.y
        
        return (abs(x - window_x) < self.wall_params.window_dimensions.x / 2 and
                abs(y - window_y) < self.wall_params.window_dimensions.y / 2)

    def create_spray_pose(self, x: float, y: float) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = self.paint_params.standoff
        # Set orientation for spray nozzle perpendicular to wall
        pose.orientation.w = 1.0
        return pose

    def plan_painting_motion(self, path: PoseArray) -> bool:
        # Configure painting constraints
        constraints = self.create_painting_constraints()
        self.move_group.set_path_constraints(constraints)
        
        # Plan path
        success = self.move_group.compute_cartesian_path(
            path.poses,
            eef_step=0.01,
            jump_threshold=0.0
        )
        
        return success

    def execute_painting_motion(self):
        try:
            # Enable force control
            self.get_logger().info('Enabling force control...')
            self.force_controller.activate()
            
            # Execute trajectory
            self.get_logger().info('Executing painting motion...')
            success = self.move_group.execute(wait=True)
            
            if not success:
                self.get_logger().error('Failed to execute painting motion')
                return False
                
            return True
            
        finally:
            self.force_controller.deactivate()
    
    def create_painting_constraints(self):
        constraints = Constraints()
        
        # Create orientation constraint for spray nozzle
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = "base_link"
        orient_constraint.link_name = "tool0"  # UR10e tool frame
        orient_constraint.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)  # Nozzle perpendicular to wall
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0

        constraints.orientation_constraints.append(orient_constraint)
        return constraints
def main():
    rclpy.init()
    planner = WallPaintingPlanner()
    rclpy.spin(planner)
    try:
        # Generate and execute painting path
        path = planner.generate_coverage_path()
        if planner.plan_painting_motion(path):
            planner.execute_painting_motion()
        else:
            planner.get_logger().error('Failed to plan painting motion')
    
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
