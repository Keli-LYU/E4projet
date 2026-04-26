#include "behavior_tree/plugins/action/compute_local_plan.hpp"

using namespace expleo_nav_stack;

ComputeLocalPlanAction::ComputeLocalPlanAction(
    const std::string & name,
    const BT::NodeConfig & conf)
    : nav2_behavior_tree::BtActionNode<Action>(name, "compute_local_plan", conf)
{
    // Note: The base class now takes the node name twice: 
    // Once for the BT instance name, and once as the ROS Action name.
}

void ComputeLocalPlanAction::on_tick()
{
    getInput("global_plan", goal_.global_path);
    getInput("planner_id", goal_.planner_id);
}

BT::NodeStatus ComputeLocalPlanAction::on_success()
{
    setOutput("local_path", result_.result->local_path);
    // Set empty error code, action was successful
    setOutput("error_code_id", ActionGoal::NONE);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeLocalPlanAction::on_aborted()
{
    nav_msgs::msg::Path empty_path;
    setOutput("local_path", empty_path);
    setOutput("error_code_id", result_.result->error_code);
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeLocalPlanAction::on_cancelled()
{
    nav_msgs::msg::Path empty_path;
    setOutput("local_path", empty_path);
    // Set empty error code, action was cancelled
    setOutput("error_code_id", ActionGoal::NONE);
    return BT::NodeStatus::SUCCESS;
}

void ComputeLocalPlanAction::halt()
{
    nav_msgs::msg::Path empty_path;
    setOutput("local_path", empty_path);
    nav2_behavior_tree::BtActionNode<Action>::halt();
}

#include "behaviortree_cpp/bt_factory.h"

// In v4, this is the standard way to export a plugin library
BT_REGISTER_NODES(factory)
{
    // Use the factory to register your node type directly
    // This is cleaner than the manual 'builder' lambda unless you have special needs
    factory.registerNodeType<expleo_nav_stack::ComputeLocalPlanAction>("ComputeLocalPlan");
}
