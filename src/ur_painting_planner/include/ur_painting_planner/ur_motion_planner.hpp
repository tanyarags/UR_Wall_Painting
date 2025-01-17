#ifndef UR_MOTION_PLANNER_HPP_
#define UR_MOTION_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace ur_motion_planner
{

class URMotionPlannerNode : public rclcpp::Node
{
public:
  explicit URMotionPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~URMotionPlannerNode();

private:
  // Constants
  static constexpr const char* GROUP_NAME = "ur_manipulator";
  static constexpr const char* TOOL_FRAME = "tool0";
  const std::string BASE_FRAME = "base_link";

  // Helper functions
  bool waitForMoveGroup();
  
  geometry_msgs::msg::PoseStamped createUR10ePose(
    double x, double y, double z,
    const std::string & frame_id);

  std::vector<geometry_msgs::msg::PoseStamped> createZigzagPattern(
    double start_x, double start_y, double start_z,
    double width, double height, int num_strokes,
    const std::string & frame_id = "base_link");

  moveit::task_constructor::Task createUR10eMotionTask(
    const std::vector<geometry_msgs::msg::PoseStamped> & stroke_points);

  // Service callbacks
  void executeMotionPlan();

  // Class members
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  rclcpp::TimerBase::SharedPtr execution_timer_;
  void initializeNode();
  rclcpp::TimerBase::SharedPtr init_timer_;
};

}  // namespace ur_motion_planner

#endif  // UR_MOTION_PLANNER_HPP_