#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

// Dedime a diky tomu mame rovnou moznost na action klienta (na vypocet trasy a jizdu)
class NavigateToPoseAction : public BT::RosActionNode<NavigateToPose>
{
public:
    NavigateToPoseAction(const std::string& name,
                         const BT::NodeConfig& conf,
                         const BT::RosNodeParams& params)
        : BT::RosActionNode<NavigateToPose>(name, conf, params)
    {}

    // Z BB vezme x a y od LookupPose
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<double>("x", "Target position X"),
            BT::InputPort<double>("y", "Target position Y")
        });
    }

    bool setGoal(Goal& goal) override
    {
        // TODO: Načtěte souřadnice x a y z input portů (getInput<double>).
        // Naplňte goal.pose: nastavte header.frame_id na "map",
        // pozici na načtené souřadnice a orientaci (w=1.0).
        // Vraťte false pokud porty nejsou dostupné, jinak true.
        auto x = getInput<double>("x");
        auto y = getInput<double>("y");

        if (!x || !y) {
            RCLCPP_ERROR(logger(), "NavigateToPoseAction: Missing x or y coordinates!");
            return false;
        }

        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = node_.lock()->now();
        goal.pose.pose.position.x = x.value();
        goal.pose.pose.position.y = y.value();
        goal.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(logger(), "NavigateToPoseAction: Sending goal [%.2f, %.2f]",
                    x.value(), y.value());
        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& wr) override
    {
        // TODO: Zkontrolujte wr.code. Pokud je SUCCEEDED, vraťte SUCCESS, jinak FAILURE.
        if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(logger(), "NavigateToPoseAction: Success!");
            return BT::NodeStatus::SUCCESS;
        }

        RCLCPP_ERROR(logger(), "NavigateToPoseAction: Failed!");
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
    {
        // TODO: Zalogujte chybu a vraťte FAILURE.
        RCLCPP_ERROR(logger(), "NavigateToPoseAction failed: %d",
                     static_cast<int>(error));
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> /*feedback*/) override
    {
        // TODO: Vraťte RUNNING (akce stále probíhá).
        return BT::NodeStatus::RUNNING;
    }
};

//Z C++ souboru udela dynamickou knihovnu (.so) --> BT Server ji pak dokaze nacist
CreateRosNodePlugin(NavigateToPoseAction, "NavigateToPoseAction");
