// point_to_point.cpp

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

using namespace moveit::task_constructor;

Task createPaintingTask(const std::vector<geometry_msgs::PoseStamped>& strokePoints) {
    Task t("painting_task");

    // Properties for the entire task
    t.setProperty("group", "ur_manipulator");
    t.setProperty("eef", "end_effector");
    t.setProperty("ik_frame", "tool_frame");

    // Create Cartesian path solver for smooth movements
    auto cartesian = std::make_shared<solvers::CartesianPath>();
    cartesian->setMaxVelocityScaling(0.1);  // Slower movements for painting
    cartesian->setMaxAccelerationScaling(0.1);

    // Initial approach to starting position
    {
        auto approach = std::make_unique<stages::Connect>(
            "approach_stroke_start",
            stages::Connect::GroupPlannerVector{{"arm", "RRTConnect"}}
        );
        t.add(std::move(approach));
    }

    // For each stroke point, create a sequence of movements
    for (size_t i = 0; i < strokePoints.size(); ++i) {
        std::string prefix = "stroke_" + std::to_string(i);
        
        // Generate target pose for this point
        {
            auto generator = std::make_unique<stages::GeneratePose>(prefix + "_target");
            generator->setPose(strokePoints[i]);
            t.add(std::move(generator));
        }

        // Move to the point using Cartesian path
        {
            auto move = std::make_unique<stages::MoveRelative>(prefix + "_execute");
            move->setGroup("arm");
            move->setSolver(cartesian);
            
            // Set motion properties
            move->setMinMaxDistance(0.001, 0.1);  // Fine control over movement
            move->setCartesianStepSize(0.01);     // Small steps for smooth motion

            t.add(std::move(move));
        }

        // Optional: Add painting-specific properties
        if (i > 0) {
            // Maintain constant force/speed during stroke
            auto properties = t.stages().back()->properties();
            properties.set("force_threshold", 10.0);  // Example force in Newtons
            properties.set("speed_factor", 0.8);      // Reduced speed during painting
        }
    }

    return t;
}

// Example usage for creating a zigzag painting pattern
std::vector<geometry_msgs::PoseStamped> createZigzagPattern(
    double start_x, double start_y, double start_z,
    double width, double height, int num_strokes) {
    
    std::vector<geometry_msgs::PoseStamped> points;
    double stroke_spacing = height / (num_strokes - 1);
    
    for (int i = 0; i < num_strokes; ++i) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "base_link";
        
        // Forward stroke
        pose.pose.position.x = start_x;
        pose.pose.position.y = start_y + (i * stroke_spacing);
        pose.pose.position.z = start_z;
        points.push_back(pose);
        
        // End of stroke
        pose.pose.position.x = start_x + width;
        points.push_back(pose);
        
        // If not the last stroke, add connecting movement
        if (i < num_strokes - 1) {
            pose.pose.position.y = start_y + ((i + 1) * stroke_spacing);
            points.push_back(pose);
        }
    }
    
    return points;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "painting_task");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create zigzag pattern
    auto strokePoints = createZigzagPattern(0.3, -0.2, 0.5, 0.4, 0.3, 5);
    
    // Create and execute task
    Task t = createPaintingTask(strokePoints);
    
    try {
        t.init();
        
        if (t.plan()) {
            t.execute();
        }
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Task failed with: " << e.what());
    }

    return 0;
}

// #include <memory>
// #include <vector>
// #pragma once
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_msgs/msg/constraints.hpp>
// #include <moveit_msgs/msg/orientation_constraint.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <geometry_msgs/msg/pose_array.hpp>
// #include <geometry_msgs/msg/vector3.hpp>
// #include <control_msgs/action/follow_cartesian_trajectory.hpp>
// #include <std_msgs/msg/bool.hpp>

// struct WallParameters {
//     double width;
//     double height;
//     geometry_msgs::msg::Pose wall_pose;
//     geometry_msgs::msg::Pose window_pose;
//     geometry_msgs::msg::Vector3 window_dimensions;
// };

// struct PaintingParameters {
//     double spray_width;
//     double overlap;
//     double standoff;
//     double speed;
// };

// class ForceController {
// public:
//     explicit ForceController(rclcpp::Node::SharedPtr node) 
//         : client_(rclcpp_action::create_client<control_msgs::action::FollowCartesianTrajectory>
//             (node, "force_control")) {}

//     void activate() {
//         auto goal = control_msgs::action::FollowCartesianTrajectory::Goal();
//         // Configure force control parameters
//         client_->async_send_goal(goal);
//     }

//     void deactivate() {
//         client_->async_cancel_all_goals();
//     }

// private:
//     rclcpp_action::Client<control_msgs::action::FollowCartesianTrajectory>::SharedPtr client_;
// };

// class SprayController {
// public:
//     explicit SprayController(rclcpp::Node::SharedPtr node) 
//         : spray_pub_(node->create_publisher<std_msgs::msg::Bool>("/spray_control", 10)) {}

//     void enable() {
//         auto msg = std_msgs::msg::Bool();
//         msg.data = true;
//         spray_pub_->publish(msg);
//     }

//     void disable() {
//         auto msg = std_msgs::msg::Bool();
//         msg.data = false;
//         spray_pub_->publish(msg);
//     }

// private:
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spray_pub_;
// };

// class WallPaintingPlanner : public rclcpp::Node {
// public:
//     WallPaintingPlanner() 
//         : Node("painting_node") {
//         setup_parameters();
//         setup_controllers();
//         move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//             shared_from_this(), "manipulator");
//     }

// private:
//     void setup_parameters() {
//         // Declare parameters
//         this->declare_parameter("wall.width", 3.0);
//         this->declare_parameter("wall.height", 2.4);
//         this->declare_parameter("window.position.x", 1.2);
//         this->declare_parameter("window.position.y", 1.5);
//         this->declare_parameter("window.dimensions.width", 0.8);
//         this->declare_parameter("window.dimensions.height", 1.0);
//         this->declare_parameter("painting.spray_width", 0.15);
//         this->declare_parameter("painting.overlap", 0.1);
//         this->declare_parameter("painting.standoff", 0.2);
//         this->declare_parameter("painting.speed", 0.3);

//         // Get parameter values
//         wall_params_.width = this->get_parameter("wall.width").as_double();
//         wall_params_.height = this->get_parameter("wall.height").as_double();
//         wall_params_.window_pose.position.x = this->get_parameter("window.position.x").as_double();
//         wall_params_.window_pose.position.y = this->get_parameter("window.position.y").as_double();
//         wall_params_.window_dimensions.x = this->get_parameter("window.dimensions.width").as_double();
//         wall_params_.window_dimensions.y = this->get_parameter("window.dimensions.height").as_double();

//         paint_params_.spray_width = this->get_parameter("painting.spray_width").as_double();
//         paint_params_.overlap = this->get_parameter("painting.overlap").as_double();
//         paint_params_.standoff = this->get_parameter("painting.standoff").as_double();
//         paint_params_.speed = this->get_parameter("painting.speed").as_double();
//     }

//     void setup_controllers() {
//         force_controller_ = std::make_unique<ForceController>(shared_from_this());
//         spray_controller_ = std::make_unique<SprayController>(shared_from_this());
//     }

//     geometry_msgs::msg::PoseArray generate_coverage_path() {
//         geometry_msgs::msg::PoseArray path;
//         double current_height = 0.0;

//         while (current_height < wall_params_.height) {
//             // Left to right
//             for (double x = 0.0; x < wall_params_.width; 
//                  x += paint_params_.spray_width * (1.0 - paint_params_.overlap)) {
//                 if (!is_window_region(x, current_height)) {
//                     path.poses.push_back(create_spray_pose(x, current_height));
//                 }
//             }

//             current_height += paint_params_.spray_width;

//             // Right to left
//             if (current_height < wall_params_.height) {
//                 for (double x = wall_params_.width; x > 0.0; 
//                      x -= paint_params_.spray_width * (1.0 - paint_params_.overlap)) {
//                     if (!is_window_region(x, current_height)) {
//                         path.poses.push_back(create_spray_pose(x, current_height));
//                     }
//                 }
//                 current_height += paint_params_.spray_width;
//             }
//         }
//         return path;
//     }

//     bool is_window_region(double x, double y) {
//         double window_x = wall_params_.window_pose.position.x;
//         double window_y = wall_params_.window_pose.position.y;

//         return (std::abs(x - window_x) < wall_params_.window_dimensions.x / 2.0 &&
//                 std::abs(y - window_y) < wall_params_.window_dimensions.y / 2.0);
//     }

//     geometry_msgs::msg::Pose create_spray_pose(double x, double y) {
//         geometry_msgs::msg::Pose pose;
//         pose.position.x = x;
//         pose.position.y = y;
//         pose.position.z = paint_params_.standoff;
//         pose.orientation.w = 1.0;
//         return pose;
//     }

//     bool plan_painting_motion(const geometry_msgs::msg::PoseArray& path) {
//         moveit_msgs::msg::Constraints constraints = create_painting_constraints();
//         move_group_->setPathConstraints(constraints);

//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         move_group_->computeCartesianPath(path.poses, 0.01, 0.0, plan.trajectory_);
        
//         return !plan.trajectory_.joint_trajectory.points.empty();
//     }

//     bool execute_painting_motion() {
//         try {
//             RCLCPP_INFO(this->get_logger(), "Enabling force control...");
//             force_controller_->activate();

//             RCLCPP_INFO(this->get_logger(), "Executing painting motion...");
//             bool success = move_group_->execute() == moveit::planning_interface::MoveItErrorCode::SUCCESS;

//             if (!success) {
//                 RCLCPP_ERROR(this->get_logger(), "Failed to execute painting motion");
//                 return false;
//             }

//             return true;
//         }
//         catch (...) {
//             force_controller_->deactivate();
//             throw;
//         }
//     }

//     moveit_msgs::msg::Constraints create_painting_constraints() {
//         moveit_msgs::msg::Constraints constraints;
//         moveit_msgs::msg::OrientationConstraint orient_constraint;

//         orient_constraint.header.frame_id = "base_link";
//         orient_constraint.link_name = "tool0";
//         orient_constraint.orientation.y = 1.0;
//         orient_constraint.absolute_x_axis_tolerance = 0.1;
//         orient_constraint.absolute_y_axis_tolerance = 0.1;
//         orient_constraint.absolute_z_axis_tolerance = 0.1;
//         orient_constraint.weight = 1.0;

//         constraints.orientation_constraints.push_back(orient_constraint);
//         return constraints;
//     }

//     WallParameters wall_params_;
//     PaintingParameters paint_params_;
//     std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
//     std::unique_ptr<ForceController> force_controller_;
//     std::unique_ptr<SprayController> spray_controller_;
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<WallPaintingPlanner>();
    
//     try {
//         auto path = node->generate_coverage_path();
//         if (node->plan_painting_motion(path)) {
//             node->execute_painting_motion();
//         } else {
//             RCLCPP_ERROR(node->get_logger(), "Failed to plan painting motion");
//         }
//     }
//     catch (const std::exception& e) {
//         RCLCPP_ERROR(node->get_logger(), "Exception occurred: %s", e.what());
//     }

//     rclcpp::shutdown();
//     return 0;
// }