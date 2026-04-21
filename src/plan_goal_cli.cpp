#include <memory>
#include <string>
#include <cstdlib>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "nav_msgs/msg/odometry.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3) {
        std::cerr << "Pouziti: ros2 run mpc_rbt_student plan_goal_cli x y [z]\n";
        return 1;
    }

    double goal_x = std::atof(argv[1]);
    double goal_y = std::atof(argv[2]);
    double goal_z = 0.0;

    if (argc >= 4) {
        goal_z = std::atof(argv[3]);
    }

    auto node = rclcpp::Node::make_shared("plan_goal_cli");

    // 1) Precti aktualni odometrii
    nav_msgs::msg::Odometry odom_msg;
    bool got_odom = rclcpp::wait_for_message(
        odom_msg,
        node,
        "/odom",
        std::chrono::seconds(3)
    );

    if (!got_odom) {
        std::cerr << "Nepodarilo se precist /odom do 3 s.\n";
        rclcpp::shutdown();
        return 1;
    }

    // 2) Klient na plan service
    auto client = node->create_client<nav_msgs::srv::GetPlan>("/plan_path");

    while (!client->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
        std::cout << "Cekam na /plan_path...\n";
    }

    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();

    // Start = aktualni poloha z /odom
    request->start.header.frame_id = "map";
    request->start.pose.position.x = odom_msg.pose.pose.position.x;
    request->start.pose.position.y = odom_msg.pose.pose.position.y;
    request->start.pose.position.z = odom_msg.pose.pose.position.z;
    request->start.pose.orientation = odom_msg.pose.pose.orientation;

    // Goal = to, co zadas
    request->goal.header.frame_id = "map";
    request->goal.pose.position.x = goal_x;
    request->goal.pose.position.y = goal_y;
    request->goal.pose.position.z = goal_z;
    request->goal.pose.orientation.w = 1.0;

    request->tolerance = 0.0;

    std::cout << "Start z /odom: ["
              << request->start.pose.position.x << ", "
              << request->start.pose.position.y << "] -> goal: ["
              << request->goal.pose.position.x << ", "
              << request->goal.pose.position.y << "]\n";

    auto future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        std::cout << "Cesta naplanovana, pocet bodu: "
                  << response->plan.poses.size() << "\n";
    } else {
        std::cerr << "Volani /plan_path selhalo.\n";
    }

    rclcpp::shutdown();
    return 0;
}
