#pragma once

#include <expleo_nav_stack/expleo_local_planner.hpp>

namespace expleo_nav_stack {

class TemplateLocalPlanner : public LocalPlanner
{
  public:
    TemplateLocalPlanner() = default;
    ~TemplateLocalPlanner() = default;

    /**
     * @param  parent pointer to user's node
     * @param  name The name of this planner
     * @param  tf A pointer to a TF buffer
     * @param  costmap_ros A pointer to the costmap
     */
    virtual void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    /**
     * @brief Method to cleanup resources used on shutdown.
     */
    virtual void cleanup() override;

    /**
     * @brief Method to active planner and any threads involved in
     * execution.
     */
    virtual void activate() override;

    /**
     * @brief Method to deactive planner and any threads involved in
     * execution.
     */
    virtual void deactivate() override;

    /**
     * @brief Method create the plan from a starting and ending goal.
     * @param global_path The global path around which the local path must
     *                    be computed
     * @return            The local path to give to the controller
     */
    virtual nav_msgs::msg::Path
    createLocalPlan(const nav_msgs::msg::Path &global_path) override;

  private:
    nav2_util::LifecycleNode::SharedPtr _node;
};
} // namespace expleo_nav_stack
