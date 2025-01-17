#include "ur_painting_planner/ur_motion_planner.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<ur_motion_planner::URMotionPlannerNode>(options);
  
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}