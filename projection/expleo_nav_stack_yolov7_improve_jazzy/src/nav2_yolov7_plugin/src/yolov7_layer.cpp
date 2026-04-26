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
*         Steve Macenski
*********************************************************************/
// Include the header file
#include "nav2_yolov7_plugin/yolov7_layer.hpp"
// Include standard libraries 
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
// Include the message definition for the detection list
#include "perception2d_interfaces/msg/detection_list.hpp"
#include "perception2d_interfaces/msg/detection_bounds_list.hpp"
// Other includes
#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "angles/angles.h"

// Use specific constants from nav2_costmap_2d 
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_yolov7_plugin
{
Yolov7Layer::~Yolov7Layer() = default;

// Function to initialize the layer
void Yolov7Layer::onInitialize()
{
  // Declare variables
  bool track_unknown_space;
  double transform_tolerance;

  // Lock the node to ensure it's still valid
  auto node = node_.lock();
  if (node) {
    // If node is valid, set the last clearing time to the current time
    last_clearing_time_ = node->now();
  } else {
    // Log an error if the node cannot be locked
    RCLCPP_ERROR(logger_, "Failed to lock node in onInitialize");
  }
  // Set the interval for clearing the costmap
  clearing_interval_ = 0.1;

  // The topics that we'll subscribe to from the parameter server
  std::string detectionlist_topic;
  std::string ros_topic;
  // TODO(mjeronimo): these four are candidates for dynamic update
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/Detection_coordinates_list")));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
  node->get_parameter(name_ + "." + "max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  node->get_parameter("track_unknown_space", track_unknown_space);
  node->get_parameter("transform_tolerance", transform_tolerance);
  node->get_parameter(name_ + "." + "observation_sources", detectionlist_topic);
  node->get_parameter(name_ + "." + "topic", ros_topic);

  rolling_window_ = layered_costmap_->isRolling();
  // Define the default value for the cost map depending on if the unknown space is tracked or not.
  if (track_unknown_space) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = FREE_SPACE;
  }

  // Match the size of the costmap to the layered costmap
  Yolov7Layer::matchSize();

  current_ = true;
  was_reset_ = false;
  global_frame_ = layered_costmap_->getGlobalFrameID();
 
  // Create a subscriber for the detection list topic
  auto detectionlist_sub = node->create_subscription<perception2d_interfaces::msg::DetectionList>(
    ros_topic, 10,
    std::bind(&Yolov7Layer::detectionListCallback, this, std::placeholders::_1));
  detectionlist_subscribers_.push_back(detectionlist_sub);

  auto detection_bounds_sub = node->create_subscription<perception2d_interfaces::msg::DetectionBoundsList>(
      "/Detection_bounds_list", 10,
      std::bind(&Yolov7Layer::detectionBoundsList, this, std::placeholders::_1));
  detection_bounds_subscribers_.push_back(detection_bounds_sub);
}


// Callback function for the detection list topic
void Yolov7Layer::detectionListCallback(
  const perception2d_interfaces::msg::DetectionList::SharedPtr msg)
{
  //RCLCPP_INFO(logger_, "Received detection list with %zu detections", msg->detection_list.size());
  //for (const auto& detection : msg->detection_list) {
    //RCLCPP_INFO(logger_, "Detection - x: %f, y: %f", detection.x, detection.y);
  //}
  // Store the detected coordinates in a variable
  detected_coords_ = msg;
}

void Yolov7Layer::detectionBoundsList(const perception2d_interfaces::msg::DetectionBoundsList::SharedPtr msg)
{
  //RCLCPP_INFO(logger_, "Received detection bounds list with %zu detections", msg->detection_bounds_list.size());
  //for (const auto& detection_bounds : msg->detection_bounds_list) {
    //RCLCPP_INFO(logger_, "Detection - x_min: %f, x_max: %f", detection_bounds.x_min, detection_bounds.x_max);
  //}
  detected_bounds_ = msg;
}

// Function to update the bounds of the costmap based on detected obstacles
void Yolov7Layer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double *min_x,
  double *min_y, double *max_x, double *max_y)
{
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    //RCLCPP_WARN(logger_, "Layer is not enabled");
    return;
  }
/*
  for (const auto &polygon : last_detections_){
    setConvexPolygonCost(polygon, nav2_costmap_2d::FREE_SPACE);
  }
  last_detections_.clear();

  // Clear the costmap if it's time to do it
  auto node = node_.lock();
  if (node) {
      rclcpp::Time now = node->now();
  if ((now - last_clearing_time_).seconds() > clearing_interval_) {
        clearCostmap();
        //clearCostmap();
        last_clearing_time_ = now;
      }
    } else {
        //RCLCPP_ERROR(logger_, "Failed to lock node in updateBounds");
        return;
    }
*/

  // Clear only behind the robot if enough time has passed
  auto node = node_.lock();
  if (node) {
      clearCostmap();
  } else {
    RCLCPP_ERROR(logger_, "Failed to lock node in updateBounds");
    return;
  }



  // Defining robot position at the centre of the local costmap
  double local_robot_x = getSizeInCellsX() / 2;
  double local_robot_y = getSizeInCellsY() / 2;

  auto transformToWorldMark = [&](geometry_msgs::msg::Point& point){
    double x = (point.x*cos(robot_yaw) - point.y*sin(robot_yaw)) + robot_x;
    double y = (point.x*sin(robot_yaw) + point.y*cos(robot_yaw)) + robot_y;
    point.x = x;
    point.y = y;
  };
  /*
  auto transformToCostmap = [&](geometry_msgs::msg::Point& point){
    transformToWorldMark(point);
    point.x = (point.x / getResolution())+local_robot_x; 
    point.y = (point.y / getResolution())+local_robot_y;
  };
  */
  // Iterate through each detected coordinate and update local bounds
  if (detected_coords_ ) {
    //if (detected_bounds_){
      for (const auto& detection : detected_coords_->detection_list) {
        for (const auto& detection_bounds : detected_bounds_->detection_bounds_list){

        //double detection_x = detection.x;
        //double detection_y = -detection.y;

        double detection_x_min = detection_bounds.x_min;
        double detection_x_max = detection_bounds.x_max;
        double detection_y_min = -detection_bounds.y_min;
        double detection_y_max = -detection_bounds.y_max;
        

        // third point > nearest point
        double detection_xx_min = detection_bounds.xx_min;
        double detection_yy_min = -detection_bounds.yy_min;
        std::string label = detection_bounds.label;
        //RCLCPP_INFO(logger_,"label: %s", label.c_str());

        if (detection_x_min < getSizeInMetersX() && detection_x_max < getSizeInMetersX() && detection_y_min < getSizeInMetersY() && detection_y_max < getSizeInMetersY() && detection_xx_min < getSizeInMetersX() && detection_yy_min < getSizeInMetersY()) {

          double dx = detection_x_max - detection_x_min;
          double dy = detection_y_max - detection_y_min;

          double wx = -dy;
          double wy = dx;

          geometry_msgs::msg::Point A, B, C, D;
          A.x = detection_x_min; A.y = detection_y_min;
          B.x = detection_x_max; B.y = detection_y_max;
          if (detection_xx_min != 0.0 && detection_yy_min != 0.0) {
            D.x = detection_xx_min; D.y = detection_yy_min;
            C.x = (A.x + B.x) - D.x; C.y = (A.y + B.y) - D.y;
          }else {
            C.x = detection_x_max + wx; C.y = detection_y_max + wy; 
            D.x = detection_x_min + wx; D.y = detection_y_min + wy;   
          }
          //RCLCPP_INFO(logger_, " %s : A (%f, %f), B (%f, %f), C (%f, %f), D (%f, %f)", label.c_str(), A.x, A.y, B.x, B.y, C.x, C.y, D.x, D.y);

          transformToWorldMark(A);
          transformToWorldMark(B);
          transformToWorldMark(C);
          transformToWorldMark(D);
          std::vector<geometry_msgs::msg::Point> square_polygon = {A,B,C,D};
          setConvexPolygonCost(square_polygon, LETHAL_OBSTACLE);
          //last_detections_.push_back(square_polygon);
          //transformToCostmap(A);
          //transformToCostmap(B);
          //transformToCostmap(C);
          //transformToCostmap(D);
          //RCLCPP_INFO(logger_, "%s", success?"True":"False");
        }
        
        }
      }
      //detected_coords_ = NULL;
      //detected_bounds_ = NULL;
    //}
  } 
  else {
    //RCLCPP_WARN(logger_, "Detected coordinates are null");
  }
  
  // Update the external bounds based on the local bounds
  *min_x = robot_x - getSizeInMetersX()/2;
  *min_y = robot_y - getSizeInMetersY()/2;
  *max_x = robot_x + getSizeInMetersX()/2;
  *max_y = robot_y + getSizeInMetersY()/2;
  // Use extra bounds for updating the costmap
  useExtraBounds(min_x,min_y ,max_x ,max_y) ;
  // Update the footprint of the robot
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}


// Function to update the footprint of the robot in the costmap
void Yolov7Layer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw, 
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!footprint_clearing_enabled_) {return;}
  nav2_costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);
  
  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}


// Function to update the cost values in the master grid
void Yolov7Layer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  // if not current due to reset, set current now after clearing
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }
  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }
  // Update the costmap based on the combination method
  switch (combination_method_) {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}
/*
// Erase 
void Yolov7Layer::clearCostmap(double robot_x, double robot_y, double robot_yaw)
{
  unsigned int size_x = getSizeInCellsX();
  unsigned int size_y = getSizeInCellsY();

  double resolution = getResolution();
  double origin_x = getOriginX();
  double origin_y = getOriginY();

  for (unsigned int i = 0; i < size_x; ++i)
  {
    for (unsigned int j = 0; j < size_y; ++j)
    {
      double wx = origin_x + i * resolution;
      double wy = origin_y + j * resolution;

      // Compute angle between robot heading and cell
      double dx = wx - robot_x;
      double dy = wy - robot_y;
      double angle = std::atan2(dy, dx);
      double angle_diff = angles::shortest_angular_distance(robot_yaw, angle);

      // If point is behind the robot
      if (std::abs(angle_diff) > M_PI_2) {
        unsigned int index = getIndex(i, j);
        costmap_[index] = nav2_costmap_2d::FREE_SPACE;
      }
    }
  }
}
*/



// Function to clear the costmap
void Yolov7Layer::clearCostmap()
{
  unsigned int size_x = getSizeInCellsX();
  unsigned int size_y = getSizeInCellsY();
  for (unsigned int i = 0; i < size_x; ++i)
  {
    for (unsigned int j = 0; j < size_y ; ++j) //for (unsigned int j = size_y*0.65; j < size_y ; ++j)
    {
      unsigned int index = getIndex(i, j);
      costmap_[index] = nav2_costmap_2d::FREE_SPACE;
    }
  }
  //RCLCPP_INFO(logger_, "Costmap cleared");
}


// Function to activate the layer
void
Yolov7Layer::activate()
{
  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->subscribe();
    }
  }
}


// Function to deactivate the layer
void
Yolov7Layer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->unsubscribe();
    }
  }
}


// Function to reset the layer
void
Yolov7Layer::reset()
{
  resetMaps();
  current_ = false;
  was_reset_ = true;
}

}

PLUGINLIB_EXPORT_CLASS(nav2_yolov7_plugin::Yolov7Layer, nav2_costmap_2d::Layer)
