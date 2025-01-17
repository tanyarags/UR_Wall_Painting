#include "ur_painting_planner/ur_motion_planner.hpp"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <chrono>
#include <thread>

namespace ur_motion_planner
{

URMotionPlannerNode::URMotionPlannerNode(const rclcpp::NodeOptions & options)
: Node("ur_motion_planner_node", options)
{
  RCLCPP_INFO(get_logger(), "Initializing URMotionPlannerNode");

  // Declare parameters
  this->declare_parameter("robot_description", "");
  this->declare_parameter("robot_description_semantic", "");
  this->declare_parameter("kinematics_yaml", "");

  // Wait for move_group capabilities
  if (!waitForMoveGroup()) {
    RCLCPP_ERROR(get_logger(), "Failed to find move_group capabilities");
    return;
  }

  // Create a timer to initialize everything after node is fully constructed
  init_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&URMotionPlannerNode::initializeNode, this));
}
void URMotionPlannerNode::initializeNode()
{
  // Cancel the init timer as we only need to run this once
  init_timer_->cancel();

  try {
    RCLCPP_INFO(get_logger(), "Starting node initialization...");

    // Get parameters
    auto robot_description = this->get_parameter("robot_description").as_string();
    auto robot_description_semantic = this->get_parameter("robot_description_semantic").as_string();
    
    if (robot_description.empty() || robot_description_semantic.empty()) {
      RCLCPP_ERROR(get_logger(), "Empty robot description parameters");
      return;
    }

    // Create planning scene monitor with more detailed error checking
    RCLCPP_INFO(get_logger(), "Creating planning scene monitor...");
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      shared_from_this(), "robot_description");
    
    if (!planning_scene_monitor_) {
      RCLCPP_ERROR(get_logger(), "Failed to create planning scene monitor");
      return;
    }

    auto scene = planning_scene_monitor_->getPlanningScene();
    if (!scene) {
      RCLCPP_ERROR(get_logger(), "Failed to get planning scene");
      return;
    }

    RCLCPP_INFO(get_logger(), "Starting planning scene monitors...");
    // Start monitors
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();

    // Wait a bit to ensure monitors are running
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(get_logger(), "Creating planning scene interface...");
    // Initialize planning scene interface
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    if (!planning_scene_interface_) {
      RCLCPP_ERROR(get_logger(), "Failed to create planning scene interface");
      return;
    }

    RCLCPP_INFO(get_logger(), "Creating execution timer...");
    // Create a timer to trigger motion planning
    execution_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&URMotionPlannerNode::executeMotionPlan, this));

    RCLCPP_INFO(get_logger(), "URMotionPlannerNode initialized successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Error during initialization: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Unknown error during initialization");
  }
}

URMotionPlannerNode::~URMotionPlannerNode()
{
  RCLCPP_INFO(get_logger(), "Shutting down URMotionPlannerNode");
}

bool URMotionPlannerNode::waitForMoveGroup()
{
  const double timeout = 10.0;  // seconds
  const double poll_interval = 0.1;  // seconds
  const std::string move_group_name = "move_group";

  auto start = this->now();
  while (rclcpp::ok()) {
    // Check if move_group is available
    bool service_exists = false;
    try {
      std::vector<std::string> node_names = get_node_names();
      for (const auto& name : node_names) {
        if (name.find(move_group_name) != std::string::npos) {
          service_exists = true;
          break;
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Error checking for move_group: %s", e.what());
    }

    if (service_exists) {
      RCLCPP_INFO(get_logger(), "move_group is available");
      return true;
    }

    if ((this->now() - start).seconds() > timeout) {
      RCLCPP_ERROR(get_logger(), "Timed out waiting for move_group");
      return false;
    }

    // Fix: Use proper duration type for sleep_for
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(poll_interval * 1000)));
  }

  return false;
}

geometry_msgs::msg::PoseStamped URMotionPlannerNode::createUR10ePose(
  double x, double y, double z,
  const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = this->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;

  // Setting tool0 orientation for vertical approach
  pose.pose.orientation.w = 0.707;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.707;
  pose.pose.orientation.z = 0.0;

  return pose;
}

std::vector<geometry_msgs::msg::PoseStamped> URMotionPlannerNode::createZigzagPattern(
  double start_x, double start_y, double start_z,
  double width, double height, int num_strokes,
  const std::string & frame_id)
{
  std::vector<geometry_msgs::msg::PoseStamped> points;
  double stroke_spacing = height / (num_strokes - 1);

  for (int i = 0; i < num_strokes; ++i) {
    // Start point of stroke
    points.push_back(
      createUR10ePose(
        start_x,
        start_y + (i * stroke_spacing),
        start_z,
        frame_id));

    // End point of stroke
    points.push_back(
      createUR10ePose(
        start_x + width,
        start_y + (i * stroke_spacing),
        start_z,
        frame_id));
  }

  return points;
}
moveit::task_constructor::Task URMotionPlannerNode::createUR10eMotionTask(
  const std::vector<geometry_msgs::msg::PoseStamped>& stroke_points)
{
  using namespace moveit::task_constructor;

  RCLCPP_INFO(get_logger(), "Creating new task");
  Task task("ur10e_motion");
  
  if (stroke_points.empty()) {
    throw std::runtime_error("No stroke points provided");
  }

  RCLCPP_INFO(get_logger(), "Setting up task pipeline");
  task.stages()->setName("UR10e Motion Pipeline");

  // Load robot model with more detailed error checking
  RCLCPP_INFO(get_logger(), "Loading robot model");
  task.loadRobotModel(shared_from_this());

  if (!task.getRobotModel()) {
    throw std::runtime_error("Failed to load robot model in task");
  }

  RCLCPP_INFO(get_logger(), "Configuring Cartesian solver");
  // Configure Cartesian solver with error checking
  auto cartesian_solver = std::make_shared<solvers::CartesianPath>();
  if (!cartesian_solver) {
    throw std::runtime_error("Failed to create Cartesian solver");
  }
  
  cartesian_solver->setMaxVelocityScalingFactor(0.3);
  cartesian_solver->setMaxAccelerationScalingFactor(0.2);
  cartesian_solver->setStepSize(0.01);

  RCLCPP_INFO(get_logger(), "Creating home position stage");
  // Add initial home position stage
  {
    auto stage = std::make_unique<stages::MoveTo>("move_to_home");
    stage->setGroup(GROUP_NAME);
    std::map<std::string, double> joint_pose = {
      {"shoulder_pan_joint", 0.0},
      {"shoulder_lift_joint", -M_PI / 2},
      {"elbow_joint", 0.0},
      {"wrist_1_joint", -M_PI / 2},
      {"wrist_2_joint", 0.0},
      {"wrist_3_joint", 0.0}
    };
    stage->setGoal(joint_pose);
    task.add(std::move(stage));
  }

  RCLCPP_INFO(get_logger(), "Creating stroke point stages");
  // Create stages for each stroke point with error checking
  for (size_t i = 0; i < stroke_points.size(); ++i) {
    std::string prefix = "stroke_" + std::to_string(i);
    RCLCPP_INFO(get_logger(), "Creating stage for stroke point %zu", i);

    try {
      // Approach point
      {
        auto approach_pose = stroke_points[i];
        approach_pose.pose.position.z += 0.05;  // 5cm above stroke point

        auto stage = std::make_unique<stages::MoveTo>(prefix + "_approach");
        stage->setGroup(GROUP_NAME);
        stage->setGoal(approach_pose);
        task.add(std::move(stage));
      }

      // Contact point
      {
        auto stage = std::make_unique<stages::MoveTo>(prefix + "_contact");
        stage->setGroup(GROUP_NAME);
        stage->setGoal(stroke_points[i]);
        task.add(std::move(stage));
      }

      // Linear motion if not the last point
      if (i < stroke_points.size() - 1) {
        auto relative_motion = std::make_unique<stages::MoveRelative>(prefix + "_stroke");
        relative_motion->setGroup(GROUP_NAME);
        relative_motion->setIKFrame(TOOL_FRAME);
        relative_motion->properties().set("planner", cartesian_solver);

        geometry_msgs::msg::Vector3Stamped direction;
        direction.header = stroke_points[i + 1].header;
        direction.vector.x = stroke_points[i + 1].pose.position.x - stroke_points[i].pose.position.x;
        direction.vector.y = stroke_points[i + 1].pose.position.y - stroke_points[i].pose.position.y;
        direction.vector.z = stroke_points[i + 1].pose.position.z - stroke_points[i].pose.position.z;

        relative_motion->setDirection(direction);
        task.add(std::move(relative_motion));
      }
    } catch (const std::exception& e) {
      throw std::runtime_error("Failed to create stage for point " + std::to_string(i) + ": " + e.what());
    }
  }

  RCLCPP_INFO(get_logger(), "Task creation completed");
  return task;
}

void URMotionPlannerNode::executeMotionPlan()
{
  try {
    RCLCPP_INFO(get_logger(), "Starting motion plan execution");

    if (!planning_scene_monitor_ || !planning_scene_monitor_->getPlanningScene()) {
      RCLCPP_ERROR(get_logger(), "Planning scene not available");
      return;
    }

    RCLCPP_INFO(get_logger(), "Creating stroke points");
    std::vector<geometry_msgs::msg::PoseStamped> stroke_points;
    try {
      stroke_points = createZigzagPattern(
        0.4,    // start_x
        -0.3,   // start_y
        0.3,    // start_z
        0.4,    // width
        0.3,    // height
        5       // num_strokes
      );
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to create zigzag pattern: %s", e.what());
      return;
    }

    RCLCPP_INFO(get_logger(), "Creating motion task");
    moveit::task_constructor::Task task;
    try {
      task = createUR10eMotionTask(stroke_points);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to create motion task: %s", e.what());
      return;
    }

    RCLCPP_INFO(get_logger(), "Setting up task pipeline");
    try {
      task.init();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize task: %s", e.what());
      return;
    }

    RCLCPP_INFO(get_logger(), "Planning motion");
    try {
      if (!task.plan()) {
        RCLCPP_ERROR(get_logger(), "Motion planning failed");
        return;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception during planning: %s", e.what());
      return;
    }

    RCLCPP_INFO(get_logger(), "Planning successful!");
    
    const auto& solutions = task.solutions();
    if (solutions.empty()) {
      RCLCPP_ERROR(get_logger(), "No solutions found!");
      return;
    }

    RCLCPP_INFO(get_logger(), "Executing solution");
    try {
      const moveit::task_constructor::SolutionBase& solution = *solutions.front();
      task.execute(solution);
      RCLCPP_INFO(get_logger(), "Execution completed successfully");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception during execution: %s", e.what());
      return;
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Task failed with: %s", e.what());
  }

  execution_timer_->cancel();
}

}  // namespace ur_motion_planner