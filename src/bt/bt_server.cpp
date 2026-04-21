#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

#include <chrono>
#include <vector>
#include <string>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_server_node");

    // Ziskani cesty k XML z parametru
    std::string xml_filename;
    node->declare_parameter("bt_xml_filename", "");
    node->get_parameter("bt_xml_filename", xml_filename);

    if (xml_filename.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'bt_xml_filename' is empty.");
        rclcpp::shutdown();
        return 1;
    }

    // Zjistime kde je balicek nainstalovanej
    std::string package_prefix = ament_index_cpp::get_package_prefix("mpc_rbt_student");
    std::string plugin_path = package_prefix + "/lib/mpc_rbt_student/";

    BT::BehaviorTreeFactory factory;

    // Seznam dynamickych knihoven, ktere chcem registrovat jako uzly
    std::vector<std::string> ros_plugins = {
        "get_task_service_plugin.so",
        "get_dropoff_service_plugin.so",
        "navigate_to_pose_action_plugin.so",
        "confirm_loading_service_plugin.so"
    };

    // Nahrani vsech pluginu, ktery potrebujou pristup k ROS (Akce/Sluzby)
    for (const auto& plugin : ros_plugins) {
        std::string full_plugin_path = plugin_path + plugin;

        // Parametry pro ROS uzel uvnitr BT pluginu
        BT::RosNodeParams params;
        params.nh = node;
        params.default_port_value = "";
        params.server_timeout = std::chrono::milliseconds(5000);

        RegisterRosNode(factory, full_plugin_path, params);
        RCLCPP_INFO(node->get_logger(), "ROS plugin načten z: %s", full_plugin_path.c_str());
    }

    // Nahrani LookupPose uzlu
    std::string lookup_plugin = plugin_path + "lookup_pose_plugin.so";
    factory.registerFromPlugin(lookup_plugin);
    RCLCPP_INFO(node->get_logger(), "BT plugin načten z: %s", lookup_plugin.c_str());

    try {
        // Vytvoreni stromu a spusteni konzoloveho logovani
        auto tree = factory.createTreeFromFile(xml_filename);
        BT::StdCoutLogger logger(tree);

        RCLCPP_INFO(node->get_logger(), "Strom vytvořen. Startuji...");

        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        rclcpp::Rate rate(10.0);

        // Hlavni smycka - tikame stromem, dokud neni konec nebo zruseno
        while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
            status = tree.tickOnce();

            // Umozni rosu zpracovat data ze sluzeb nebo akci
            rclcpp::spin_some(node);
            rate.sleep();
        }

        RCLCPP_INFO(node->get_logger(), "Hotovo. Status: %s", BT::toStr(status).c_str());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "BT Error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
