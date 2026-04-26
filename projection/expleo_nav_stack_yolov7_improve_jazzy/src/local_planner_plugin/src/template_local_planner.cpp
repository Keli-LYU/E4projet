#include "local_planner_plugin/template_local_planner.hpp"

namespace expleo_nav_stack {

void TemplateLocalPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    _node = parent.lock();
}

/**
 * @brief Method to cleanup resources used on shutdown.
 */
void TemplateLocalPlanner::cleanup()
{
    RCLCPP_INFO(_node->get_logger(), "Cleaning up plugin TemplateLocalPlanner");
}

/**
 * @brief Method to active planner and any threads involved in
 * execution.
 */
void TemplateLocalPlanner::activate()
{
    RCLCPP_INFO(_node->get_logger(), "Activating plugin TemplateLocalPlanner");
}

/**
 * @brief Method to deactive planner and any threads involved in
 * execution.
 */
void TemplateLocalPlanner::deactivate()
{

    RCLCPP_INFO(_node->get_logger(),
                "Deactivating plugin TemplateLocalPlanner");
}

/**
 * @brief Method create the plan from a starting and ending goal.
 * @param global_path The global path around which the local path must
 *                    be computed
 * @return            The local path to give to the controller
 */
nav_msgs::msg::Path
TemplateLocalPlanner::createLocalPlan(const nav_msgs::msg::Path &global_path)
{
    RCLCPP_INFO(_node->get_logger(), "Calling createLocalPlan");
    auto new_path = global_path;
    auto time = _node->get_clock()->now();
    new_path.header.stamp.sec = time.seconds();
    new_path.header.stamp.nanosec = time.nanoseconds();
    return new_path;
}
} // namespace expleo_nav_stack

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(expleo_nav_stack::TemplateLocalPlanner,
                       expleo_nav_stack::LocalPlanner)
