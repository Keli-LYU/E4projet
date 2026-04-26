// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EXPLEO_NAV_STACK__LOCAL_PLANNER_HPP_
#define EXPLEO_NAV_STACK__LOCAL_PLANNER_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include <memory>
#include <string>

namespace expleo_nav_stack
{

    /**
     * @class LocalPlanner
     * @brief Abstract interface for local planners to adhere to with pluginlib
     */
    class LocalPlanner
    {
      public:
        using Ptr = std::shared_ptr<LocalPlanner>;

        /**
         * @brief Virtual destructor
         */
        virtual ~LocalPlanner() {}

        /**
         * @param  parent pointer to user's node
         * @param  name The name of this planner
         * @param  tf A pointer to a TF buffer
         * @param  costmap_ros A pointer to the costmap
         */
        virtual void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

        /**
         * @brief Method to cleanup resources used on shutdown.
         */
        virtual void cleanup() = 0;

        /**
         * @brief Method to active planner and any threads involved in
         * execution.
         */
        virtual void activate() = 0;

        /**
         * @brief Method to deactive planner and any threads involved in
         * execution.
         */
        virtual void deactivate() = 0;

        /**
         * @brief Method create the plan from a starting and ending goal.
         * @param global_path The global path around which the local path must
         *                    be computed
         * @return            The local path to give to the controller
         */
        virtual nav_msgs::msg::Path
        createLocalPlan(const nav_msgs::msg::Path &global_path) = 0;
    };

} // namespace expleo_nav_stack

#endif // EXPLEO_NAV_STACK__LOCAL_PLANNER_HPP_
