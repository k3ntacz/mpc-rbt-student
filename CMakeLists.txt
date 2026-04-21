cmake_minimum_required(VERSION 3.5)
project(mpc_rbt_student)

# --- Nastavení kompilátoru a standardů ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# --- Vyhledání balíčků ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mpc_rbt_simulator REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# --- Definice závislostí projektu (přejmenovaná proměnná) ---
set(PROJECT_MAIN_DEPENDENCIES
    rclcpp
    rclcpp_action
    sensor_msgs
    nav_msgs
    nav2_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    mpc_rbt_simulator
    geometry_msgs
    std_srvs
    behaviortree_cpp
    behaviortree_ros2
)

include_directories(include)

# --- Makro pro definici Behavior Tree pluginů (přesunuto výše) ---
macro(add_bt_plugin plugin_name source_file)
    add_library(${plugin_name} SHARED ${source_file})
    ament_target_dependencies(${plugin_name}
        behaviortree_ros2
        behaviortree_cpp
        rclcpp
        rclcpp_action
        nav2_msgs
        std_srvs
    )
    target_include_directories(${plugin_name} PRIVATE include)
    target_compile_definitions(${plugin_name} PRIVATE BT_PLUGIN_EXPORT)
    set_target_properties(${plugin_name} PROPERTIES PREFIX "")
endmacro()

# --- Kompilace Behavior Tree pluginů ---
add_bt_plugin(get_task_service_plugin src/bt/GetTaskService.cpp)
add_bt_plugin(confirm_loading_service_plugin src/bt/ConfirmLoadingService.cpp)
add_bt_plugin(get_dropoff_service_plugin src/bt/GetDropoffService.cpp)
add_bt_plugin(navigate_to_pose_action_plugin src/bt/NavigateToPoseAction.cpp)
add_bt_plugin(lookup_pose_plugin src/bt/LookupPose.cpp)

# --- Kompilace hlavních spustitelných uzlů ---
# Core navigace a lokalizace
add_executable(localization src/localization_node.cpp src/Localization.cpp)
ament_target_dependencies(localization ${PROJECT_MAIN_DEPENDENCIES})

add_executable(planning src/planning_node.cpp src/Planning.cpp)
ament_target_dependencies(planning ${PROJECT_MAIN_DEPENDENCIES})

add_executable(motion_control src/motion_control_node.cpp src/MotionControl.cpp)
ament_target_dependencies(motion_control ${PROJECT_MAIN_DEPENDENCIES})

# Warehouse management a BT Server
add_executable(bt_server src/bt/bt_server.cpp)
ament_target_dependencies(bt_server ${PROJECT_MAIN_DEPENDENCIES} ament_index_cpp)

add_executable(warehouse_manager src/bt/WarehouseManager.cpp src/bt/warehouse_manager_node.cpp)
ament_target_dependencies(warehouse_manager ${PROJECT_MAIN_DEPENDENCIES})

# Pomocné utility
add_executable(keyboard_control src/keyboard_control_node.cpp src/KeyboardControl.cpp)
ament_target_dependencies(keyboard_control ${PROJECT_MAIN_DEPENDENCIES})

add_executable(plan_goal_cli src/plan_goal_cli.cpp)
ament_target_dependencies(plan_goal_cli ${PROJECT_MAIN_DEPENDENCIES})

# --- Instalační pravidla (sloučená a reorganizovaná) ---

# Binární soubory (uzly) a knihovny (pluginy)
install(TARGETS
    localization
    planning
    motion_control
    bt_server
    warehouse_manager
    keyboard_control
    plan_goal_cli
    get_task_service_plugin
    confirm_loading_service_plugin
    get_dropoff_service_plugin
    navigate_to_pose_action_plugin
    lookup_pose_plugin
    DESTINATION lib/${PROJECT_NAME}
)

# Adresáře s daty
install(DIRECTORY 
    behavior_trees 
    config 
    launch 
    rviz
    DESTINATION share/${PROJECT_NAME}
)

# --- Exporty balíčku ---
ament_export_include_directories(include)
ament_package()