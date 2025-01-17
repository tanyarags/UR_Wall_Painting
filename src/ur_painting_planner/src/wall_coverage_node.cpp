// wall_coverage_node.cpp
#include "wall_coverage_planner.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("wall_coverage_node");
    
    WallCoveragePlanner planner(node);
    
    // Define wall surface
    geometry_msgs::msg::Pose surface_origin;
    surface_origin.position.x = 1.0;  // 1m in front of robot
    surface_origin.position.y = 0.0;
    surface_origin.position.z = 0.5;  // Starting height
    
    // Generate and execute coverage trajectory
    auto grid = planner.decomposeSurface(surface_origin, 2.0, 2.0);  // 2x2m wall
    auto trajectory = planner.generateTrajectory(grid);
    planner.executeTrajectory(trajectory);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}