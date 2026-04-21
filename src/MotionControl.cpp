#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"
#include "tf2/utils.h"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

        current_pose_.pose.orientation.w = 1.0;
        current_pose_.header.frame_id = "map";

        // Subscribers for odometry and laser scans
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));
        
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));

        // Publisher for robot control
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        // Client for path planning
        plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");

        // Action server
        nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
            this,
            "/go_to_goal",
            std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
            std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "Motion control node started.");

        while(!plan_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
            RCLCPP_INFO(get_logger(), "Cekam na planovaci sluzbu /plan_path...");
        }

        RCLCPP_INFO(get_logger(), "Planovaci sluzba je pripravena!");
    }

void MotionControlNode::checkCollision() {
    if (laser_scan_.ranges.empty()) return;

    bool hrozi_srazka = false;

    for (size_t i = 0; i < laser_scan_.ranges.size(); ++i) {
        double angle = laser_scan_.angle_min + i * laser_scan_.angle_increment;
        
        // Normalizace uhlu
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;

        if (std::abs(angle) < 0.5) {
            double distance = laser_scan_.ranges[i];
            // Vynecháme neplatné hodnoty (0 nebo inf)
            if (distance > 0.05 && distance < 0.4) {
                hrozi_srazka = true;
                break;
            }
        }
    }

    if (hrozi_srazka && goal_handle_ && goal_handle_->is_active()) {
        RCLCPP_ERROR(this->get_logger(), "NEBEZPECI KOLIZE! Zastavuji motor a rusim akci.");
        geometry_msgs::msg::Twist stop_msg;
        twist_publisher_->publish(stop_msg);
        
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        goal_handle_->abort(result);
    }
}

void MotionControlNode::updateTwist() {
    if (path_.poses.empty()) return;

    if (std::abs(current_pose_.pose.orientation.x) < 1e-6 &&
        std::abs(current_pose_.pose.orientation.y) < 1e-6 &&
        std::abs(current_pose_.pose.orientation.z) < 1e-6 &&
        std::abs(current_pose_.pose.orientation.w) < 1e-6) 
    {
        return; 
    }

    auto target_pose = path_.poses.front().pose.position;

    double current_x = current_pose_.pose.position.x;
    double current_y = current_pose_.pose.position.y;
    
    tf2::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double dx = target_pose.x - current_x;
    double dy = target_pose.y - current_y;
    double distance = std::hypot(dx, dy);

    if (distance < 0.35){ 
        path_.poses.erase(path_.poses.begin());
        return;
    }

    // Ochrana atan2(0,0) - pokud jsme extrémně blízko, neotáčíme se
    double target_yaw = (distance < 0.001) ? yaw : std::atan2(dy, dx);

    double angle_diff = target_yaw - yaw;
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.angular.z = 0.5 * angle_diff; // Zvýšeno P pro rychlejší reakci

    if(std::abs(angle_diff) > 0.8){
        cmd_vel.linear.x = 0.0;
    } else {
        cmd_vel.linear.x = 0.4 * (1.0 - std::abs(angle_diff)/0.8); 
    }

    twist_publisher_->publish(cmd_vel);
} 

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    (void)uuid; (void)goal; // Odstranění warningů
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    goal_handle_ = goal_handle;
    auto action_goal = goal_handle->get_goal();
    
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = current_pose_;
    request->goal = action_goal->pose;
    
    RCLCPP_INFO(this->get_logger(), "Cil prijat. Ptam se planovace na cestu...");
    plan_client_->async_send_request(request, std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
}

void MotionControlNode::execute() {
    RCLCPP_INFO(this->get_logger(), "Cekam na vypocet trasy od planovace...");

    rclcpp::Rate loop_rate(10); 
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
    auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();

    while (rclcpp::ok() && path_.poses.empty()) {
        if (goal_handle_->is_canceling() || !goal_handle_->is_active()) return;
        loop_rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "Trasa dorazila, vyrazime vpřed!");

    while (rclcpp::ok()){
        if (goal_handle_->is_canceling()){
            geometry_msgs::msg::Twist stop_msg;
            twist_publisher_->publish(stop_msg);
            goal_handle_->canceled(result);
            return;
        }

        checkCollision();

        if (!goal_handle_->is_active()) return;

        if (path_.poses.empty()){
            geometry_msgs::msg::Twist stop_msg;
            twist_publisher_->publish(stop_msg);
            goal_handle_->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Dorazili jsme do cile!");
            return;
        }

        goal_handle_->publish_feedback(feedback);
        updateTwist(); 
        loop_rate.sleep();
    }
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    auto response = future.get();
    if (response && response->plan.poses.size() > 0) {
        path_ = response->plan;
        std::thread(&MotionControlNode::execute, this).detach();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Planovac nenasel cestu!");
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        if (goal_handle_) goal_handle_->abort(result);
    }
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    if (std::isnan(msg.pose.pose.position.x) || std::isnan(msg.pose.pose.orientation.w)) return;

    current_pose_.header = msg.header;
    current_pose_.header.frame_id = "map"; 
    current_pose_.pose = msg.pose.pose;
    
    // Pozor: Tento offset -0.5 může způsobovat skákání v RViz, pokud 
    // static_transform_publisher v launch souboru dělá to samé.
    // Pokud robot v RViz "bliká", tento řádek smaž.
    current_pose_.pose.position.x -= 0.5;
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    laser_scan_ = msg;
}
