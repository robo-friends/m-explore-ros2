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

#include <thread>

#include <map_merge/map_merge.h>
#include <map_merge/ros1_names.hpp>
#include <rcpputils/asserts.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace map_merge
{
MapMerge::MapMerge() : Node("map_merge", rclcpp::NodeOptions()
                                       .allow_undeclared_parameters(true)
                                       .automatically_declare_parameters_from_overrides(true)),
subscriptions_size_(0),
logger_(this->get_logger())
{
  std::string frame_id;
  std::string merged_map_topic;

  if (!this->has_parameter("merging_rate")) this->declare_parameter<double>("merging_rate", 4.0);
  if (!this->has_parameter("discovery_rate")) this->declare_parameter<double>("discovery_rate", 0.05);
  if (!this->has_parameter("estimation_rate")) this->declare_parameter<double>("estimation_rate", 0.5);
  if (!this->has_parameter("known_init_poses")) this->declare_parameter<bool>("known_init_poses", true);
  if (!this->has_parameter("estimation_confidence")) this->declare_parameter<double>("estimation_confidence", 1.0);
  if (!this->has_parameter("robot_map_topic")) this->declare_parameter<std::string>("robot_map_topic", "map");
  if (!this->has_parameter("robot_map_updates_topic")) this->declare_parameter<std::string>("robot_map_updates_topic", "map_updates");
  if (!this->has_parameter("robot_namespace")) this->declare_parameter<std::string>("robot_namespace", "");
  if (!this->has_parameter("merged_map_topic")) this->declare_parameter<std::string>("merged_map_topic", "map");
  if (!this->has_parameter("world_frame")) this->declare_parameter<std::string>("world_frame", "world");

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

  // Timers
  map_merging_timer_ = this->create_wall_timer(
    std::chrono::milliseconds((uint16_t)(1000.0 / merging_rate_)),
    [this]() { mapMerging(); });
  // execute right away to simulate the ros1 first while loop on a thread
  mapMerging();

  topic_subscribing_timer_ = this->create_wall_timer(
    std::chrono::milliseconds((uint16_t)(1000.0 / discovery_rate_)),
    [this]() { topicSubscribing(); });

  // For topicSubscribing() we need to spin briefly for the discovery to happen
  rclcpp::Rate r(100);
  int i = 0;
  while (rclcpp::ok() && i < 100) {
    rclcpp::spin_some(this->get_node_base_interface());
    r.sleep();
    i++;
  }
  topicSubscribing();

  if (!have_initial_poses_){
    pose_estimation_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / estimation_rate_)),
      [this]() { poseEstimation(); });
    // execute right away to simulate the ros1 first while loop on a thread
    poseEstimation();
  }
}

/*
 * Subscribe to pose and map topics
 */
void MapMerge::topicSubscribing()
{
  RCLCPP_DEBUG(logger_, "Robot discovery started.");
  RCLCPP_INFO_ONCE(logger_, "Robot discovery started.");

  // ros::master::V_TopicInfo topic_infos;
  geometry_msgs::msg::Transform init_pose;
  std::string robot_name;
  std::string map_topic;
  std::string map_updates_topic;

  // ros::master::getTopics(topic_infos);
  std::map<std::string, std::vector<std::string>> topic_infos = this->get_topic_names_and_types();

  for (const auto& topic_it : topic_infos) {
    std::string topic_name = topic_it.first;
    std::vector<std::string> topic_types = topic_it.second;
    // iterate over all topic types
    for (const auto& topic_type : topic_types) {
      // RCLCPP_INFO(logger_, "Topic: %s, type: %s", topic_name.c_str(), topic_type.c_str());

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

      RCLCPP_INFO(logger_, "adding robot [%s] to system", robot_name.c_str());
      {
        // We don't lock since because of ROS2 default executor only a callback can run at a time
        // std::lock_guard<boost::shared_mutex> lock(subscriptions_mutex_);
        subscriptions_.emplace_front();
        ++subscriptions_size_;
      }

      // no locking here. robots_ are used only in this procedure
      MapSubscription& subscription = subscriptions_.front();
      robots_.insert({robot_name, &subscription});
      subscription.initial_pose = init_pose;

      /* subscribe callbacks */
      map_topic = ros1_names::append(robot_name, robot_map_topic_);
      map_updates_topic =
          ros1_names::append(robot_name, robot_map_updates_topic_);
      RCLCPP_INFO(logger_, "Subscribing to MAP topic: %s.", map_topic.c_str());
      auto map_qos = rclcpp::QoS(rclcpp::KeepLast(50)).transient_local().reliable();
      subscription.map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          map_topic, map_qos,
          [this, &subscription](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            fullMapUpdate(msg, subscription);
          });
      RCLCPP_INFO(logger_, "Subscribing to MAP updates topic: %s.",
              map_updates_topic.c_str());
      subscription.map_updates_sub =
          this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
              map_updates_topic, map_qos,
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
  RCLCPP_INFO_ONCE(logger_, "Map merging started.");

  if (have_initial_poses_) {
    // TODO: attempt fix for SLAM toolbox: add method for padding grids to same size

    std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> grids;
    std::vector<geometry_msgs::msg::Transform> transforms;
    grids.reserve(subscriptions_size_);
    {
      // We don't lock since because of ROS2 default executor only a callback can run
      // boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);
      for (auto& subscription : subscriptions_) {
        // std::lock_guard<std::mutex> s_lock(subscription.mutex);

        grids.push_back(subscription.readonly_map);
        transforms.push_back(subscription.initial_pose);
      }
    }


    // we don't need to lock here, because when have_initial_poses_ is true we
    // will not run concurrently on the pipeline
    pipeline_.feed(grids.begin(), grids.end());
    pipeline_.setTransforms(transforms.begin(), transforms.end());
  }

  // nav_msgs::OccupancyGridPtr merged_map;
  nav_msgs::msg::OccupancyGrid::SharedPtr merged_map;
  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    merged_map = pipeline_.composeGrids(logger_);
  }
  if (!merged_map) {
    // RCLCPP_INFO(logger_, "No map merged");
    return;
  }

  RCLCPP_DEBUG(logger_, "all maps merged, publishing");
  // RCLCPP_INFO(logger_, "all maps merged, publishing");
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
  RCLCPP_INFO_ONCE(logger_, "Grid pose estimation started.");
  std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> grids;
  grids.reserve(subscriptions_size_);

  {
    // We don't lock since because of ROS2 default executor only a callback can run
    // boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);
    for (auto& subscription : subscriptions_) {
      // std::lock_guard<std::mutex> s_lock(subscription.mutex);
      grids.push_back(subscription.readonly_map);
    }
  }

  // Print grids size
  // RCLCPP_INFO(logger_, "Grids size: %d", grids.size());

  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  pipeline_.feed(grids.begin(), grids.end());
  // TODO allow user to change feature type
  bool success = pipeline_.estimateTransforms(logger_,combine_grids::FeatureType::AKAZE,
                               confidence_threshold_);
  // bool success = pipeline_.estimateTransforms(logger_, combine_grids::FeatureType::SURF,
  //                              confidence_threshold_);
  // bool success = pipeline_.estimateTransforms(logger_, combine_grids::FeatureType::ORB,
  //                              confidence_threshold_);
  if (!success) {
    RCLCPP_INFO(logger_, "No grid poses estimated");
  }
}

// void MapMerge::fullMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr& msg,
//                              MapSubscription& subscription)
void MapMerge::fullMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr msg,
                     MapSubscription& subscription)
{
  RCLCPP_DEBUG(logger_, "received full map update");
  // RCLCPP_INFO(logger_, "received full map update");
  std::lock_guard<std::mutex> lock(subscription.mutex);
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

  nav_msgs::msg::OccupancyGrid::SharedPtr map;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr readonly_map;  // local copy
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
  return ros1_names::parentNamespace(topic);
}

/* identifies topic via suffix */
bool MapMerge::isRobotMapTopic(const std::string topic, std::string type)
{
  /* test whether topic is robot_map_topic_ */
  std::string topic_namespace = ros1_names::parentNamespace(topic);
  bool is_map_topic = ros1_names::append(topic_namespace, robot_map_topic_) == topic;

  // /* test whether topic contains *anywhere* robot namespace */
  auto pos = topic.find(robot_namespace_);
  bool contains_robot_namespace = pos != std::string::npos;

  // /* we support only occupancy grids as maps */
  bool is_occupancy_grid = type == "nav_msgs/msg/OccupancyGrid";

  // /* we don't want to subscribe on published merged map */
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
  std::string merging_namespace = ros1_names::append(name, "map_merge");
  double yaw = 0.0;

  bool success =
      this->get_parameter(ros1_names::append(merging_namespace, "init_pose_x"),
                      pose.translation.x) &&
      this->get_parameter(ros1_names::append(merging_namespace, "init_pose_y"),
                      pose.translation.y) &&
      this->get_parameter(ros1_names::append(merging_namespace, "init_pose_z"),
                      pose.translation.z) &&
      this->get_parameter(ros1_names::append(merging_namespace, "init_pose_yaw"),
                      yaw);

  tf2::Quaternion q;
  q.setEuler(0., 0., yaw);
  pose.rotation = toMsg(q);

  return success;
}
}  // namespace map_merge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  // // this package is still in development -- start with debugging enabled
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //                                    ros::console::levels::Debug)) {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  // ROS2 code
  auto node = std::make_shared<map_merge::MapMerge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
