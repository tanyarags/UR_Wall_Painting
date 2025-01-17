// wall_coverage_planner.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>
#include <cmath>

class WallCoveragePlanner {
public:
    WallCoveragePlanner(const rclcpp::Node::SharedPtr& node)
        : node_(node),
          move_group_(node, "ur10e_manipulator") {
        trajectory_pub_ = node_->create_publisher<nav_msgs::msg::Path>
            ("coverage_trajectory", 10);
    }

    struct SurfaceCell {
        geometry_msgs::msg::Pose pose;
        bool covered = false;
        bool is_obstacle = false;
    };

    std::vector<std::vector<SurfaceCell>> decomposeSurface(
        const geometry_msgs::msg::Pose& surface_origin,
        double width, double height) {
        std::vector<std::vector<SurfaceCell>> grid;
        const double base_cell_size = 0.1;
        
        int rows = std::ceil(height / base_cell_size);
        int cols = std::ceil(width / base_cell_size);
        
        grid.resize(rows);
        for (auto& row : grid) {
            row.resize(cols);
        }

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                auto& cell = grid[i][j];
                cell.pose = surface_origin;
                cell.pose.position.y += i * base_cell_size;
                cell.pose.position.x += j * base_cell_size;
            }
        }
        return grid;
    }

    nav_msgs::msg::Path generateTrajectory(
        const std::vector<std::vector<SurfaceCell>>& grid) {
        nav_msgs::msg::Path path;
        path.header.frame_id = "base_link";
        
        bool reverse = false;
        for (size_t i = 0; i < grid.size(); i++) {
            if (reverse) {
                for (int j = grid[i].size()-1; j >= 0; j--) {
                    if (!grid[i][j].is_obstacle) {
                        geometry_msgs::msg::PoseStamped pose;
                        pose.pose = grid[i][j].pose;
                        path.poses.push_back(pose);
                    }
                }
            } else {
                for (size_t j = 0; j < grid[i].size(); j++) {
                    if (!grid[i][j].is_obstacle) {
                        geometry_msgs::msg::PoseStamped pose;
                        pose.pose = grid[i][j].pose;
                        path.poses.push_back(pose);
                    }
                }
            }
            reverse = !reverse;
        }
        smoothTrajectory(path);
        return path;
    }

    bool executeTrajectory(const nav_msgs::msg::Path& path) {
        move_group_.setMaxVelocityScalingFactor(0.1);
        move_group_.setMaxAccelerationScalingFactor(0.1);
        move_group_.setPlanningTime(10.0);
        move_group_.setNumPlanningAttempts(10);
        move_group_.setGoalPositionTolerance(0.01);
        move_group_.setGoalOrientationTolerance(0.01);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        for (const auto& pose : path.poses) {
            move_group_.setPoseTarget(pose.pose);
            
            auto error_code = move_group_.plan(plan);
            if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                auto execution_result = move_group_.execute(plan);
                if (execution_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                    RCLCPP_ERROR(node_->get_logger(), "Execution failed with code: %d", execution_result.val);
                    return false;
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Planning failed with code: %d", error_code.val);
                return false;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
        return true;
    }

private:
    void smoothTrajectory(nav_msgs::msg::Path& path) {
        for (size_t i = 1; i < path.poses.size() - 1; i++) {
            auto& prev = path.poses[i-1].pose;
            auto& curr = path.poses[i].pose;
            auto& next = path.poses[i+1].pose;
            
            curr.position.x = 0.25*prev.position.x + 
                            0.5*curr.position.x + 
                            0.25*next.position.x;
            curr.position.y = 0.25*prev.position.y + 
                            0.5*curr.position.y + 
                            0.25*next.position.y;
            curr.position.z = 0.25*prev.position.z + 
                            0.5*curr.position.z + 
                            0.25*next.position.z;
        }
    }

    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
};