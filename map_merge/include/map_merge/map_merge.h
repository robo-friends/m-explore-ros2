/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Zhi Yan.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez.
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

#ifndef MAP_MERGE_H_
#define MAP_MERGE_H_

#include <atomic>
#include <forward_list>
#include <mutex>
#include <unordered_map>

#include <combine_grids/merging_pipeline.h>
#include <geometry_msgs/msg/transform.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/thread.hpp>

namespace map_merge
{
struct MapSubscription {
  // protects consistency of writable_map and readonly_map
  // also protects reads and writes of shared_ptrs
  std::mutex mutex;

  geometry_msgs::msg::Transform initial_pose;
  nav_msgs::msg::OccupancyGrid::SharedPtr writable_map;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr readonly_map;

  // ros::Subscriber map_sub;
  // ros::Subscriber map_updates_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr map_updates_sub;
};

class MapMerge : public rclcpp::Node
{
private:
  /* parameters */
  double merging_rate_;
  double discovery_rate_;
  double estimation_rate_;
  double confidence_threshold_;
  std::string robot_map_topic_;
  std::string robot_map_updates_topic_;
  std::string robot_namespace_;
  std::string world_frame_;
  bool have_initial_poses_;

  // publishing
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_map_publisher_;
  // maps robots namespaces to maps. does not own
  std::unordered_map<std::string, MapSubscription*> robots_;
  // owns maps -- iterator safe
  std::forward_list<MapSubscription> subscriptions_;
  size_t subscriptions_size_;
  boost::shared_mutex subscriptions_mutex_;
  combine_grids::MergingPipeline pipeline_;
  std::mutex pipeline_mutex_;

  rclcpp::Logger logger_;

  // timers
  rclcpp::TimerBase::SharedPtr map_merging_timer_;
  rclcpp::TimerBase::SharedPtr topic_subscribing_timer_;
  rclcpp::TimerBase::SharedPtr pose_estimation_timer_;

  std::string robotNameFromTopic(const std::string& topic);
  // bool isRobotMapTopic(const ros::master::TopicInfo& topic);
  bool isRobotMapTopic(const std::string topic, std::string type);
  bool getInitPose(const std::string& name, geometry_msgs::msg::Transform& pose);

  void fullMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr msg,
                     MapSubscription& map);
  void partialMapUpdate(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg,
                        MapSubscription& map);

public:
  MapMerge();

  void topicSubscribing();
  void mapMerging();
  /**
   * @brief Estimates initial positions of grids
   * @details Relevant only if initial poses are not known
   */
  void poseEstimation();
};

}  // namespace map_merge

#endif /* MAP_MERGE_H_ */
