/* Copyright Expleo */

#pragma once
#include "expleo_nav_msgs/action/compute_local_plan.hpp"
#include <expleo_nav_stack/expleo_local_planner.hpp>
#include <nav2_planner/planner_server.hpp>

namespace expleo_nav_stack
{

    class ExpleoPlannerServer : public nav2_planner::PlannerServer
    {
      public:
        explicit ExpleoPlannerServer();
        ~ExpleoPlannerServer();

      protected:
        /**
         * @brief Configure member variables and initializes planner
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */

        nav2_util::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &state) override;
        /**
         * @brief Activate member variables
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */

        nav2_util::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &state) override;
        /**
         * @brief Deactivate member variables
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        nav2_util::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &state) override;

        /**
         * @brief Reset member variables
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        nav2_util::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &state) override;

        //-----------------------------------------------------------------------------
        // BAD COPY-PASTE: the three following template functions are
        // copy-pasted from the original repo because the template
        // implementation is located inside the source code of the
        // PlannerServer, so the template can't be expanded from our class.
        //-----------------------------------------------------------------------------
        /**
         * @brief Check if an action server is valid / active
         * @param action_server Action server to test
         * @return SUCCESS or FAILURE
         **/
        template <typename T>
        bool isLocalServerInactive(
            std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server)
        {
            if (action_server == nullptr ||
                !action_server->is_server_active()) {
                RCLCPP_DEBUG(
                    get_logger(),
                    "Action server unavailable or inactive. Stopping.");
                return true;
            }

            return false;
        }

        /**
         * @brief Check if an action server has a cancellation request
         * pending
         * @param action_server Action server to test
         * @return SUCCESS or FAILURE
         **/
        template <typename T>
        bool isLocalCancelRequested(
            std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server)
        {
            if (action_server->is_cancel_requested()) {
                RCLCPP_INFO(get_logger(),
                            "Goal was canceled. Canceling planning action.");
                action_server->terminate_all();
                return true;
            }

            return false;
        }

        /**
         * @brief Check if an action server has a preemption request and
         * replaces the goal with the new preemption goal.
         * @param action_server Action server to get updated goal if required
         * @param goal Goal to overwrite
         **/
        template <typename T>
        void getLocalPreemptedGoalIfRequested(
            std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server,
            typename std::shared_ptr<const typename T::Goal> goal)
        {
            if (action_server->is_preempt_requested()) {
                goal = action_server->accept_pending_goal();
            }
        }

        using ActionLocalPlan = expleo_nav_msgs::action::ComputeLocalPlan;
        using ActionServerLocalPlan =
            nav2_util::SimpleActionServer<ActionLocalPlan>;

        // action server to implement the ComputeLocalPlan action
        std::unique_ptr<ActionServerLocalPlan> action_server_local_plan_;

        void computeLocalPlan();
        nav_msgs::msg::Path getLocalPlan(const nav_msgs::msg::Path &global_path,
                                         const std::string &local_planner_id);

        // Local Costmap
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_costmap_ros_;
        std::unique_ptr<nav2_util::NodeThread> local_costmap_thread_;
        nav2_costmap_2d::Costmap2D *local_costmap_;

        // TF buffer
        std::shared_ptr<tf2_ros::Buffer> local_tf_;

        void waitForLocalCostmap();

        rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
            local_plan_publisher_;

        void publishLocalPlan(const nav_msgs::msg::Path &path);

        using LocalPlannerMap =
            std::unordered_map<std::string, LocalPlanner::Ptr>;
        LocalPlannerMap local_planners_;
        pluginlib::ClassLoader<LocalPlanner> lp_loader_;

        std::vector<std::string> default_local_ids_;
        std::vector<std::string> local_planner_ids_;
        std::vector<std::string> local_planner_types_;
        std::string local_planner_ids_concat_;
    };

} // namespace expleo_nav_stack
