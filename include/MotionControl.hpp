#ifndef MOTIONCTRL_HPP
#define MOTIONCTRL_HPP

#include <vector>
#include <cmath>
#include <thread>
#include <atomic>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class MotionControlNode : public rclcpp::Node {
public:
    MotionControlNode();
    // Destruktor pro ukončení vlákna
    ~MotionControlNode() {
        if (execution_thread_.joinable()) {
            execution_thread_.join();
        }
    }

private:
    // Parametry
    double goal_tolerance_ = 0.07;
    double waypoint_tolerance_ = 0.1;
    double max_linear_speed_ = 0.30;
    double max_angular_speed_ = 1.20;
    double angular_gain_ = 3.0;
    double stop_distance_ = 0.05;
    int lookahead_step_ = 5;

    // Stavové proměnné
    std::atomic<bool> executing_{false};
    std::atomic<bool> collision_detected_{false};
    size_t current_waypoint_index_ = 0;
    bool odom_ready_ = false;

    // Vlákno pro běh akce
    std::thread execution_thread_;

    // Metody
    void checkCollision();
    void updateTwist();
    void execute();
    void stopRobot();

    // Action Server Handlers
    rclcpp_action::GoalResponse navHandleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal);

    rclcpp_action::CancelResponse navHandleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle);

    void navHandleAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle);

    // Callbacks
    void pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future);
    void odomCallback(const nav_msgs::msg::Odometry & msg);
    void lidarCallback(const sensor_msgs::msg::LaserScan & msg);

    // ROS Komponenty
    rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr plan_client_;
    rclcpp_action::Server<nav2_msgs::action::NavigateToPose>::SharedPtr nav_server_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    // Action Handle
    std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle_;

    // Data
    nav_msgs::msg::Path path_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    sensor_msgs::msg::LaserScan laser_scan_;
    
};

#endif // MOTIONCTRL_HPP
