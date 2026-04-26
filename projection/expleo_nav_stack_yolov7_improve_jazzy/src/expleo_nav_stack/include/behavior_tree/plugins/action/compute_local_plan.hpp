#ifndef EXPLEO_NAVSTACK__BT__PLUGINS__ACTION__COMPUTE_LOCAL_PLAN_ACTION_HPP_
#define EXPLEO_NAVSTACK__BT__PLUGINS__ACTION__COMPUTE_LOCAL_PLAN_ACTION_HPP_

#include <string>

#include "expleo_nav_msgs/action/compute_local_plan.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav_msgs/msg/path.hpp"

namespace expleo_nav_stack
{
    /**
     ** @brief A nav2_behavior_tree::BtActionNode class that wraps
     ** nav2_msgs::action::ComputePathToPose
     **/
    class ComputeLocalPlanAction
        : public nav2_behavior_tree::BtActionNode<
              expleo_nav_msgs::action::ComputeLocalPlan>
    {
        using Action = expleo_nav_msgs::action::ComputeLocalPlan;
        using ActionResult = Action::Result;
        using ActionGoal = Action::Goal;

      public:
        /**
         ** @brief A constructor for expleo_nav_stack::ComputeLocalPlanAction
         ** @param xml_tag_name Name for the XML tag for this node
         ** @param action_name Action name this node creates a client for
         ** @param conf BT node configuration
         **/
        ComputeLocalPlanAction(const std::string &name,
                               const BT::NodeConfig &conf); // Changed signature and Type
        /**
         ** @brief Function to perform some user-defined operation on tick
         **/
        void on_tick() override;

        /**
         ** @brief Function to perform some user-defined operation upon
         ** successful completion of the action
         **/
        BT::NodeStatus on_success() override;

        /**
         ** @brief Function to perform some user-defined operation upon
         ** abortion of the action
         **/
        BT::NodeStatus on_aborted() override;

        /**
         ** @brief Function to perform some user-defined operation upon
         ** cancellation of the action
         **/
        BT::NodeStatus on_cancelled() override;

        /**
         ** \brief Override required by the a BT action. Cancel the action
         ** and set the path output
         **/
        void halt() override;

        /**
         ** @brief Creates list of BT ports
         ** @return BT::PortsList Containing basic ports along with
         ** node-specific ports
         **/
        static BT::PortsList providedPorts()
        {
            return providedBasicPorts({
                BT::InputPort<nav_msgs::msg::Path>(
                    "global_plan",
                    "Global plan around which a local plan must be computed"),
                BT::InputPort<std::string>(
                    "planner_id", "",
                    "Mapped name to the planner plugin type to use"),
                BT::OutputPort<nav_msgs::msg::Path>(
                    "local_path", "Path created by ComputeLocalPlan node"),
                BT::OutputPort<ActionResult::_error_code_type>(
                    "error_code_id", "The compute local plan error code"),
            });
        }
    };

} // namespace expleo_nav_stack

#endif // EXPLEO_NAVSTACK__BT__PLUGINS__ACTION__COMPUTE_LOCAL_PLAN_ACTION_HPP_
