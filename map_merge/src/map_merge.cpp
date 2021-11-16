/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Zhi Yan.
 *  Copyright (c) 2015-2016, Jiri Horner.
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

#include <thread>

#include <map_merge/map_merge.h>
#include <rcpputils/asserts.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


namespace map_merge
{
MapMerge::MapMerge() : Node("map_merge"),
subscriptions_size_(0)
{
  std::string frame_id;
  std::string merged_map_topic;

  this->declare_parameter<double>("merging_rate", 4.0);
  this->declare_parameter<double>("discovery_rate", 0.05);
  this->declare_parameter<double>("estimation_rate", 0.5);
  this->declare_parameter<bool>("known_init_poses", true);
  this->declare_parameter<double>("estimation_confidence", 1.0);
  this->declare_parameter<std::string>("robot_map_topic", "map");
  this->declare_parameter<std::string>("robot_map_updates_topic", "map_updates");
  this->declare_parameter<std::string>("robot_namespace", "");
  this->declare_parameter<std::string>("merged_map_topic", "map");
  this->declare_parameter<std::string>("world_frame", "world");

  this->get_parameter("merging_rate", merging_rate_);
  this->get_parameter("discovery_rate", discovery_rate_);
  this->get_parameter("estimation_rate", estimation_rate_);
  this->get_parameter("known_init_poses", have_initial_poses_);
  this->get_parameter("estimation_confidence", confidence_threshold_);
  this->get_parameter("robot_map_topic", robot_map_topic_);
  this->get_parameter("robot_map_updates_topic", robot_map_updates_topic_);
  this->get_parameter("robot_namespace", robot_namespace_);
  this->get_parameter("merged_map_topic", merged_map_topic);
  this->get_parameter("world_frame", world_frame_);


  /* publishing */
  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  merged_map_publisher_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>(merged_map_topic, 
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

/*
 * Subcribe to pose and map topics
 */
void MapMerge::topicSubscribing()
{
  RCLCPP_DEBUG(logger_, "Robot discovery started.");

  // ros::master::V_TopicInfo topic_infos;
  geometry_msgs::msg::Transform init_pose;
  std::string robot_name;
  std::string map_topic;
  std::string map_updates_topic;

  // ROS1 comment: default msg constructor does no properly initialize quaternion
  // I think in ROS2 it DOES initialize properly
  // init_pose.rotation.w = 1;  // create identity quaternion

  RCLCPP_INFO(logger_, "Getting all topics");
  // ros::master::getTopics(topic_infos);
  std::map<std::string, std::vector<std::string>> topic_infos = this->get_topic_names_and_types();

  for (const auto& topic_it : topic_infos) {
    std::string topic_name = topic_it.first;
    std::vector<std::string> topic_types = topic_it.second;
    // iterate over all topic types
    for (const auto& topic_type : topic_types) {
      RCLCPP_INFO(logger_, "Topic: %s, type: %s", topic_name.c_str(), topic_type.c_str());

      // we check only map topic
      if (!isRobotMapTopic(topic_name, topic_type)) {
        continue;
      }
    
      robot_name = robotNameFromTopic(topic_name);
      if (robots_.count(robot_name)) {
        // we already know this robot
        continue;
      }

      if (have_initial_poses_ && !getInitPose(robot_name, init_pose)) {
        RCLCPP_WARN(logger_, "Couldn't get initial position for robot [%s]\n"
                "did you defined parameters map_merge/init_pose_[xyz]? in robot "
                "namespace? If you want to run merging without known initial "
                "positions of robots please set `known_init_poses` parameter "
                "to false. See relevant documentation for details.",
                robot_name.c_str());
        continue;
      }

      // TODO: FIX THIS compile error with mutex :/
      // RCLCPP_INFO(logger_, "adding robot [%s] to system", robot_name.c_str());
      // {
      //   std::lock_guard<boost::shared_mutex> lock(subscriptions_mutex_);
      //   subscriptions_.emplace_front();
      //   ++subscriptions_size_;
      // }

      // no locking here. robots_ are used only in this procedure
      MapSubscription& subscription = subscriptions_.front();
      robots_.insert({robot_name, &subscription});
      subscription.initial_pose = init_pose;

      /* subscribe callbacks */
      map_topic = append(robot_name, robot_map_topic_);
      map_updates_topic =
          append(robot_name, robot_map_updates_topic_);
      RCLCPP_INFO(logger_, "Subscribing to MAP topic: %s.", map_topic.c_str());
      subscription.map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          map_topic, 50,
          [this, &subscription](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            fullMapUpdate(msg, subscription);
          });
      RCLCPP_INFO(logger_, "Subscribing to MAP updates topic: %s.",
              map_updates_topic.c_str());
      subscription.map_updates_sub =
          this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
              map_updates_topic, 50,
              [this, &subscription](
                  const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
                partialMapUpdate(msg, subscription);
              });
    }
  }
}

/*
 * mapMerging()
 */
void MapMerge::mapMerging()
{
  RCLCPP_DEBUG(logger_, "Map merging started.");

  // TODO: FIX COMPILING THIS
  // if (have_initial_poses_) {
  //   // std::vector<nav_msgs::OccupancyGridConstPtr> grids;
  //   std::vector<nav_msgs::msg::OccupancyGrid::ConstPtr> grids;
  //   std::vector<geometry_msgs::msg::Transform> transforms;
  //   grids.reserve(subscriptions_size_);
  //   {
  //     boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);
  //     for (auto& subscription : subscriptions_) {
  //       std::lock_guard<std::mutex> s_lock(subscription.mutex);
  //       grids.push_back(subscription.readonly_map);
  //       transforms.push_back(subscription.initial_pose);
  //     }
  //   }
  //   // we don't need to lock here, because when have_initial_poses_ is true we
  //   // will not run concurrently on the pipeline
  //   pipeline_.feed(grids.begin(), grids.end());
  //   pipeline_.setTransforms(transforms.begin(), transforms.end());
  // }

  // nav_msgs::OccupancyGridPtr merged_map;
  nav_msgs::msg::OccupancyGrid::Ptr merged_map;
  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    merged_map = pipeline_.composeGrids();
  }
  if (!merged_map) {
    return;
  }

  RCLCPP_DEBUG(logger_, "all maps merged, publishing");
  auto now = this->now();
  merged_map->info.map_load_time = now;
  merged_map->header.stamp = now;
  merged_map->header.frame_id = world_frame_;

  rcpputils::assert_true(merged_map->info.resolution > 0.f);
  merged_map_publisher_->publish(*merged_map);
}

void MapMerge::poseEstimation()
{
  RCLCPP_DEBUG(logger_, "Grid pose estimation started.");
  // TODO: FIX COMPILING THIS
  // // std::vector<nav_msgs::OccupancyGridConstPtr> grids;
  // std::vector<nav_msgs::msg::OccupancyGrid::ConstPtr> grids;
  // grids.reserve(subscriptions_size_);
  // {
  //   boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);
  //   for (auto& subscription : subscriptions_) {
  //     std::lock_guard<std::mutex> s_lock(subscription.mutex);
  //     grids.push_back(subscription.readonly_map);
  //   }
  // }

  // std::lock_guard<std::mutex> lock(pipeline_mutex_);
  // pipeline_.feed(grids.begin(), grids.end());
  // // TODO allow user to change feature type
  // pipeline_.estimateTransforms(combine_grids::FeatureType::AKAZE,
  //                              confidence_threshold_);
}

// void MapMerge::fullMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr& msg,
//                              MapSubscription& subscription)
void MapMerge::fullMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr msg,
                     MapSubscription& subscription)
{
  RCLCPP_DEBUG(logger_, "received full map update");
  std::lock_guard<std::mutex> lock(subscription.mutex);
  // if (subscription.readonly_map &&
  //     subscription.readonly_map->header.stamp > msg->header.stamp) {
  if (subscription.readonly_map){
    // ros2 header .stamp don't support > operator, we need to create them explicitly
    auto t1 = rclcpp::Time(subscription.readonly_map->header.stamp);
    auto t2 = rclcpp::Time(msg->header.stamp);
    if (t1 > t2) {
      // we have been overrunned by faster update. our work was useless.
      return;
    }
  }

  subscription.readonly_map = msg;
  subscription.writable_map = nullptr;
}

// void MapMerge::partialMapUpdate(
//     const map_msgs::OccupancyGridUpdate::ConstPtr& msg,
//     MapSubscription& subscription)
void MapMerge::partialMapUpdate(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg,
                        MapSubscription& subscription)
{
  RCLCPP_DEBUG(logger_, "received partial map update");

  if (msg->x < 0 || msg->y < 0) {
    RCLCPP_ERROR(logger_, "negative coordinates, invalid update. x: %d, y: %d", msg->x,
              msg->y);
    return;
  }

  size_t x0 = static_cast<size_t>(msg->x);
  size_t y0 = static_cast<size_t>(msg->y);
  size_t xn = msg->width + x0;
  size_t yn = msg->height + y0;

  // nav_msgs::OccupancyGridPtr map;
  // nav_msgs::OccupancyGridConstPtr readonly_map;  // local copy
  nav_msgs::msg::OccupancyGrid::Ptr map;
  nav_msgs::msg::OccupancyGrid::ConstPtr readonly_map;  // local copy
  {
    // load maps
    std::lock_guard<std::mutex> lock(subscription.mutex);
    map = subscription.writable_map;
    readonly_map = subscription.readonly_map;
  }

  if (!readonly_map) {
    RCLCPP_WARN(logger_, "received partial map update, but don't have any full map to "
             "update. skipping.");
    return;
  }

  // // we don't have partial map to take update, we must copy readonly map and
  // // update new writable map
  if (!map) {
    map.reset(new nav_msgs::msg::OccupancyGrid(*readonly_map));
  }

  size_t grid_xn = map->info.width;
  size_t grid_yn = map->info.height;

  if (xn > grid_xn || x0 > grid_xn || yn > grid_yn || y0 > grid_yn) {
    RCLCPP_WARN(logger_, "received update doesn't fully fit into existing map, "
             "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
             "map is: [0, %lu], [0, %lu]",
             x0, xn, y0, yn, grid_xn, grid_yn);
  }

  // update map with data
  size_t i = 0;
  for (size_t y = y0; y < yn && y < grid_yn; ++y) {
    for (size_t x = x0; x < xn && x < grid_xn; ++x) {
      size_t idx = y * grid_xn + x;  // index to grid for this specified cell
      map->data[idx] = msg->data[i];
      ++i;
    }
  }
  // update time stamp
  map->header.stamp = msg->header.stamp;

  {
    // store back updated map
    std::lock_guard<std::mutex> lock(subscription.mutex);
    // if (subscription.readonly_map &&
    //     subscription.readonly_map->header.stamp > map->header.stamp) {
    //   // we have been overrunned by faster update. our work was useless.
    //   return;
    // }
    if (subscription.readonly_map){
      // ros2 header .stamp don't support > operator, we need to create them explicitly
      auto t1 = rclcpp::Time(subscription.readonly_map->header.stamp);
      auto t2 = rclcpp::Time(map->header.stamp);
      if (t1 > t2) {
        // we have been overrunned by faster update. our work was useless.
        return;
      }
    }
    subscription.writable_map = map;
    subscription.readonly_map = map;
  }
}

std::string MapMerge::robotNameFromTopic(const std::string& topic)
{
  return parentNamespace(topic);
}

/* identifies topic via suffix */
bool MapMerge::isRobotMapTopic(const std::string topic, std::string type)
{
  /* test whether topic is robot_map_topic_ */
  std::string topic_namespace = parentNamespace(topic);
  bool is_map_topic = append(topic_namespace, robot_map_topic_) == topic;

  // /* test whether topic contains *anywhere* robot namespace */
  auto pos = topic.find(robot_namespace_);
  bool contains_robot_namespace = pos != std::string::npos;

  // /* we support only occupancy grids as maps */
  bool is_occupancy_grid = type == "nav_msgs/OccupancyGrid";

  // /* we don't want to subcribe on published merged map */
  bool is_our_topic = merged_map_publisher_->get_topic_name() == topic;

  return is_occupancy_grid && !is_our_topic && contains_robot_namespace &&
         is_map_topic;
}

/*
 * Get robot's initial position
 */
bool MapMerge::getInitPose(const std::string& name,
                           geometry_msgs::msg::Transform& pose)
{
  std::string merging_namespace = append(name, "map_merge");
  double yaw = 0.0;

  bool success =
      this->get_parameter(append(merging_namespace, "init_pose_x"),
                      pose.translation.x) &&
      this->get_parameter(append(merging_namespace, "init_pose_y"),
                      pose.translation.y) &&
      this->get_parameter(append(merging_namespace, "init_pose_z"),
                      pose.translation.z) &&
      this->get_parameter(append(merging_namespace, "init_pose_yaw"),
                      yaw);

  tf2::Quaternion q;
  q.setEuler(0., 0., yaw);
  pose.rotation = toMsg(q);

  return success;
}

/*
 * execute()
 */
void MapMerge::executemapMerging()
{
  // ros::Rate r(merging_rate_);
  // while (node_.ok()) {
  //   mapMerging();
  //   r.sleep();
  // }
}

void MapMerge::executetopicSubscribing()
{
  // ros::Rate r(discovery_rate_);
  // while (node_.ok()) {
  //   topicSubscribing();
  //   r.sleep();
  // }
}

void MapMerge::executeposeEstimation()
{
  // if (have_initial_poses_)
  //   return;

  // ros::Rate r(estimation_rate_);
  // while (node_.ok()) {
  //   poseEstimation();
  //   r.sleep();
  // }
}

/*
 * spin()
 */
void MapMerge::spin()
{
  // ros::spinOnce();
  // std::thread merging_thr([this]() { executemapMerging(); });
  // std::thread subscribing_thr([this]() { executetopicSubscribing(); });
  // std::thread estimation_thr([this]() { executeposeEstimation(); });
  // ros::spin();
  // estimation_thr.join();
  // merging_thr.join();
  // subscribing_thr.join();
}

}  // namespace map_merge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  // // this package is still in development -- start wil debugging enabled
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //                                    ros::console::levels::Debug)) {
  //   ros::console::notifyLoggerLevelsChanged();
  // }
  // map_merge::MapMerge map_merging;
  // map_merging.spin();
  
  // ROS2 code
  auto node = std::make_shared<map_merge::MapMerge>();
  node->topicSubscribing();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
