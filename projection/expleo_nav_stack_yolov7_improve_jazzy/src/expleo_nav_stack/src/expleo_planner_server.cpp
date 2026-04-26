#include "expleo_nav_stack/expleo_planner_server.hpp"
#include <nav2_util/node_utils.hpp>

namespace expleo_nav_stack
{

    ExpleoPlannerServer::ExpleoPlannerServer()
        : nav2_planner::PlannerServer(),
          lp_loader_("expleo_nav_stack", "expleo_nav_stack::LocalPlanner"),
          default_local_ids_{"EmptyLocalPlanner"}
    {
        // Declare this node's parameters
        declare_parameter("local_planner_plugins", default_local_ids_);
        get_parameter("local_planner_plugins", local_planner_ids_);

        // Setup the local costmap
        local_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
            "local_costmap",  // The name of the costmap
            true              // use_sim_time: Set to true if you are using CARLA/Gazebo
        );
        // Launch a thread to run the costmap node
        local_costmap_thread_ =
            std::make_unique<nav2_util::NodeThread>(local_costmap_ros_);
    }

    ExpleoPlannerServer::~ExpleoPlannerServer()
    {
        local_costmap_thread_.reset();
    }

    nav2_util::CallbackReturn
    ExpleoPlannerServer::on_configure(const rclcpp_lifecycle::State &state)
    {
        // call superclass function to configure the nav2_planner::PlannerServer
        auto config_ret = nav2_planner::PlannerServer::on_configure(state);

        if (config_ret != nav2_util::CallbackReturn::SUCCESS)
            return config_ret;

        // create our local planner server
        action_server_local_plan_ = std::make_unique<ActionServerLocalPlan>(
            shared_from_this(), "compute_local_plan",
            std::bind(&ExpleoPlannerServer::computeLocalPlan, this));

        // configure local costmap
        local_costmap_ros_->on_configure(state);
        local_costmap_ = local_costmap_ros_->getCostmap();

        local_tf_ = local_costmap_ros_->getTfBuffer();

        // configure local planner plugins
        local_planner_types_.resize(local_planner_ids_.size());

        auto node = shared_from_this();

        for (size_t i = 0; i != planner_ids_.size(); i++) {
            try {
                local_planner_types_[i] = nav2_util::get_plugin_type_param(
                    node, local_planner_ids_[i]);

                LocalPlanner::Ptr local_planner =
                    lp_loader_.createUniqueInstance(local_planner_types_[i]);

                RCLCPP_INFO(get_logger(),
                            "Created local planner plugin %s of type %s",
                            local_planner_ids_[i].c_str(),
                            local_planner_types_[i].c_str());

                local_planner->configure(node, planner_ids_[i], tf_,
                                         local_costmap_ros_);
                local_planners_.insert({local_planner_ids_[i], local_planner});
            } catch (const pluginlib::PluginlibException &ex) {
                RCLCPP_FATAL(get_logger(),
                             "Failed to create local planner. Exception: %s",
                             ex.what());
                return nav2_util::CallbackReturn::FAILURE;
            }
        }

        for (size_t i = 0; i != local_planner_ids_.size(); i++) {
            local_planner_ids_concat_ +=
                local_planner_ids_[i] + std::string(" ");
        }

        RCLCPP_INFO(get_logger(),
                    "Planner Server has %s local planners available.",
                    local_planner_ids_concat_.c_str());

        local_plan_publisher_ =
            create_publisher<nav_msgs::msg::Path>("local_planner_path", 1);

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    ExpleoPlannerServer::on_activate(const rclcpp_lifecycle::State &state)
    {
        // activate our local planner action server
        action_server_local_plan_->activate();
        local_costmap_ros_->on_activate(state);
        local_plan_publisher_->on_activate();

        // activate local planner plugins
        LocalPlannerMap::iterator it;
        for (it = local_planners_.begin(); it != local_planners_.end(); ++it) {
            it->second->activate();
        }

        // activate the nav2_planner::PlannerServer
        return nav2_planner::PlannerServer::on_activate(state);
    }

    nav2_util::CallbackReturn
    ExpleoPlannerServer::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        // deactivate our local planner action server
        action_server_local_plan_->deactivate();
        local_costmap_ros_->on_deactivate(state);
        local_plan_publisher_->on_deactivate();

        // deactivate local planner plugins
        LocalPlannerMap::iterator it;
        for (it = local_planners_.begin(); it != local_planners_.end(); ++it) {
            it->second->deactivate();
        }

        // deactivate the nav2_planner::PlannerServer
        return nav2_planner::PlannerServer::on_deactivate(state);
    }

    nav2_util::CallbackReturn
    ExpleoPlannerServer::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        action_server_local_plan_.reset();
        local_costmap_ros_->on_cleanup(state);
        local_costmap_ = nullptr;
        local_plan_publisher_.reset();

        // cleanup local planner plugins
        LocalPlannerMap::iterator it;
        for (it = local_planners_.begin(); it != local_planners_.end(); ++it) {
            it->second->cleanup();
        }
        local_planners_.clear();

        return nav2_planner::PlannerServer::on_cleanup(state);
    }

    void ExpleoPlannerServer::waitForLocalCostmap()
    {
        // Don't compute a plan until costmap is valid (after clear costmap)
        rclcpp::Rate r(100);
        while (!local_costmap_ros_->isCurrent()) {
            r.sleep();
        }
    }

    //-----------------------------------------------------------------------------
    // Callback for the action server
    //-----------------------------------------------------------------------------
    void ExpleoPlannerServer::computeLocalPlan()
    {
        // Initialize the ComputeLocalPlan goal and result
        auto goal = action_server_local_plan_->get_current_goal();
        auto result = std::make_shared<ActionLocalPlan::Result>();

        try {
            if (isLocalServerInactive(action_server_local_plan_) ||
                isLocalCancelRequested(action_server_local_plan_)) {
                return;
            }

            waitForLocalCostmap();

            getLocalPreemptedGoalIfRequested(action_server_local_plan_, goal);

            result->local_path =
                getLocalPlan(goal->global_path, goal->planner_id);

            // if (!validatePath(action_server_pose_, goal_pose, result->path,
            //                  goal->planner_id)) {
            //    return;
            //}

            // Publish the plan for visualization purposes
            publishLocalPlan(result->local_path);

            // auto cycle_duration = steady_clock_.now() - start_time;
            // result->planning_time = cycle_duration;

            // if (max_planner_duration_ &&
            //    cycle_duration.seconds() > max_planner_duration_) {
            //    RCLCPP_WARN(get_logger(),
            //                "Planner loop missed its desired rate of %.4f Hz.
            //                " "Current loop rate is %.4f Hz", 1 /
            //                max_planner_duration_, 1 /
            //                cycle_duration.seconds());
            //}

            action_server_local_plan_->succeeded_current(result);
        } catch (std::exception &ex) {
            RCLCPP_WARN(get_logger(),
                        "%s plugin failed to local plan calculation to \"%s\"",
                        goal->planner_id.c_str(), ex.what());
            action_server_local_plan_->terminate_current();
        }
    }

    //-----------------------------------------------------------------------------
    // Call the plugin createLocalPlan function to realize the local planning
    //-----------------------------------------------------------------------------
    nav_msgs::msg::Path
    ExpleoPlannerServer::getLocalPlan(const nav_msgs::msg::Path &global_path,
                                      const std::string &local_planner_id)
    {
        RCLCPP_DEBUG(get_logger(), "Computing a local path around global path");

        if (local_planners_.find(local_planner_id) != local_planners_.end()) {
            return local_planners_[local_planner_id]->createLocalPlan(
                global_path);
        } else {
            if (local_planners_.size() == 1 && local_planner_id.empty()) {
                RCLCPP_WARN_ONCE(get_logger(),
                                 "No planners specified in action call. "
                                 "Server will use only plugin %s in server."
                                 " This warning will appear once.",
                                 local_planner_ids_concat_.c_str());
                return local_planners_[local_planners_.begin()->first]
                    ->createLocalPlan(global_path);
            } else {
                RCLCPP_ERROR(get_logger(),
                             "planner %s is not a valid local planner. "
                             "Planner names are: %s",
                             local_planner_id.c_str(),
                             local_planner_ids_concat_.c_str());
            }
        }

        return nav_msgs::msg::Path();
    }

    void ExpleoPlannerServer::publishLocalPlan(const nav_msgs::msg::Path &path)
    {
        auto msg = std::make_unique<nav_msgs::msg::Path>(path);
        if (local_plan_publisher_->is_activated() &&
            local_plan_publisher_->get_subscription_count() > 0) {
            local_plan_publisher_->publish(std::move(msg));
        }
    }

} // namespace expleo_nav_stack
