/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
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
 *   * Neither the name of the Jiri Horner nor the names of its
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
 *********************************************************************/

#ifndef COSTMAP_CLIENT_
#define COSTMAP_CLIENT_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace explore
{
class Costmap2DClient
{
public:
  /**
   * @brief Contructs client and start listening
   * @details Constructor will block until first map update is received and
   * map is ready to use, also will block before trasformation
   * robot_base_frame <-> global_frame is available.
   *
   * @param node node handle to retrieve parameters from
   * @param tf_listener Will be used for transformation of robot pose.
   */
  Costmap2DClient(rclcpp::Node& node, const tf2_ros::Buffer* tf_listener);
  /**
   * @brief Get the pose of the robot in the global frame of the costmap
   * @return pose of the robot in the global frame of the costmap
   */
  geometry_msgs::msg::Pose getRobotPose() const;

  /**
   * @brief Return a pointer to the "master" costmap which receives updates from
   * all the layers.
   *
   * This pointer will stay the same for the lifetime of Costmap2DClient object.
   */
  nav2_costmap_2d::Costmap2D* getCostmap()
  {
    return &costmap_;
  }

  /**
   * @brief Return a pointer to the "master" costmap which receives updates from
   * all the layers.
   *
   * This pointer will stay the same for the lifetime of Costmap2DClient object.
   */
  const nav2_costmap_2d::Costmap2D* getCostmap() const
  {
    return &costmap_;
  }

  /**
   * @brief  Returns the global frame of the costmap
   * @return The global frame of the costmap
   */
  const std::string& getGlobalFrameID() const
  {
    return global_frame_;
  }

  /**
   * @brief  Returns the local frame of the costmap
   * @return The local frame of the costmap
   */
  const std::string& getBaseFrameID() const
  {
    return robot_base_frame_;
  }

protected:
  void updateFullMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void updatePartialMap(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg);

  nav2_costmap_2d::Costmap2D costmap_;
  bool costmap_received_ = false;  ///< @brief Flag indicating whether costmap
                                   ///< callback has been called

  const tf2_ros::Buffer* const tf_;  ///< @brief Used for transforming
                                     /// point clouds
  rclcpp::Node& node_;
  std::string global_frame_;      ///< @brief The global frame for the costmap
  std::string robot_base_frame_;  ///< @brief The frame_id of the robot base
  double transform_tolerance_;    ///< timeout before transform errors

private:
  // will be unsubscribed at destruction
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr
      costmap_updates_sub_;
};

}  // namespace explore

#endif
