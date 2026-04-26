/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, 2013, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*         David V. Lu!!
*********************************************************************/
#ifndef NAV2_YOLOV7_PLUGIN__YOLOV7_LAYER_HPP_
#define NAV2_YOLOV7_PLUGIN__YOLOV7_LAYER_HPP_
 
#include <memory>
#include <string>
#include <vector>
 
#include "rclcpp/rclcpp.hpp"
#include "laser_geometry/laser_geometry.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#include "perception2d_interfaces/msg/detection_list.hpp"
#include "perception2d_interfaces/msg/detection_bounds_list.hpp"
#pragma GCC diagnostic pop
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/footprint.hpp"
 
namespace nav2_yolov7_plugin
{
 
/**
* @class ObstacleLayer
* @brief Takes in laser and pointcloud data to populate into 2D costmap
*/
class Yolov7Layer : public nav2_costmap_2d::CostmapLayer
{
public:
  /**
   * @brief A constructor
   */
  Yolov7Layer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
  }
 
  /**
   * @brief A destructor
   */
  virtual ~Yolov7Layer();
  /**
   * @brief Initialization process of layer on startup
   */
  virtual void onInitialize();
  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;
  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  /**
   * @brief Clear this costmap
   */
  virtual void clearCostmap();
  
/*
  virtual void clearCostmap(
    double robot_x, double robot_y, double robot_yaw
  );
*/
  /**
   * @brief Deactivate the layer
   */
  virtual void deactivate();
 
  /**
   * @brief Activate the layer
   */
  virtual void activate();
 
  /**
   * @brief Reset this costmap
   */
  virtual void reset();
 
  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable() {return true;}
 
  /**
  * @brief  A callback to handle buffering LaserScan messages
  * @param message The message returned from a message notifier
  * @param buffer A pointer to the observation buffer to update
  */
  void detectionListCallback(
    const perception2d_interfaces::msg::DetectionList::SharedPtr msg
  );

  void detectionBoundsList(
    const perception2d_interfaces::msg::DetectionBoundsList::SharedPtr msg
  );

  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
  bool footprint_clearing_enabled_;
  /**
   * @brief Clear costmap layer info below the robot's footprint
   */
  void updateFootprint(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
 
  std::string global_frame_;  ///< @brief The global frame for the costmap
  double max_obstacle_height_;  ///< @brief Max Obstacle Height
 
 
 
  bool rolling_window_;
  bool was_reset_;
  int combination_method_;
 
protected:
 
  std::vector<rclcpp::Subscription<perception2d_interfaces::msg::DetectionList>::SharedPtr> detectionlist_subscribers_;
  perception2d_interfaces::msg::DetectionList::SharedPtr detected_coords_;

  // Add a new subscriber vector for DetectionBoundsList
  std::vector<rclcpp::Subscription<perception2d_interfaces::msg::DetectionBoundsList>::SharedPtr> detection_bounds_subscribers_;
  perception2d_interfaces::msg::DetectionBoundsList::SharedPtr detected_bounds_;

  std::vector<std::shared_ptr<message_filters::Subscriber<perception2d_interfaces::msg::DetectionList>>> observation_subscribers_;
  std::vector<std::shared_ptr<message_filters::Subscriber<perception2d_interfaces::msg::DetectionBoundsList>>> detection_bounds_observation_subscribers_;

  std::vector<std::vector<geometry_msgs::msg::Point>> last_detections_;

  rclcpp::Time last_clearing_time_;
  double clearing_interval_;
 
};
 
}  // namespace nav2_costmap_2d
 
#endif  // NAV2_COSTMAP_2D__OBSTACLE_LAYER_HPP_