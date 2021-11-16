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

#include <ctype.h>
bool isValidCharInName(char c)
{
  if (isalnum(c) || c == '/' || c == '_')
  {
    return true;
  }

  return false;
}

bool validate(const std::string& name, std::string& error)
{
  if (name.empty())
  {
    return true;
  }

  // First element is special, can be only ~ / or alpha
  char c = name[0];
  if (!isalpha(c) && c != '/' && c != '~')
  {
    std::stringstream ss;
    ss << "Character [" << c << "] is not valid as the first character in Graph Resource Name [" << name << "].  Valid characters are a-z, A-Z, / and in some cases ~.";
    error = ss.str();
    return false;
  }

  for (size_t i = 1; i < name.size(); ++i)
  {
    c = name[i];
    if (!isValidCharInName(c))
    {
      std::stringstream ss;
      ss << "Character [" << c << "] at element [" << i << "] is not valid in Graph Resource Name [" << name <<"].  Valid characters are a-z, A-Z, 0-9, / and _.";
      error = ss.str();

      return false;
    }
  }

  return true;
}

class InvalidNameException : public std::runtime_error
{
public:
  InvalidNameException(const std::string& msg)
  : std::runtime_error(msg)
  {}
};

std::string parentNamespace(const std::string& name)
{
  std::string error;
  if (!validate(name, error))
  {
        throw InvalidNameException(error);
  }

  if (!name.compare(""))  return "";
  if (!name.compare("/")) return "/"; 

  std::string stripped_name;

  // rstrip trailing slash
  if (name.find_last_of('/') == name.size()-1)
    stripped_name = name.substr(0, name.size() -2);
  else
    stripped_name = name;

  //pull everything up to the last /
  size_t last_pos = stripped_name.find_last_of('/');
  if (last_pos == std::string::npos)
  {
    return "";
  }
  else if (last_pos == 0)
    return "/";
  return stripped_name.substr(0, last_pos);
}

std::string clean(const std::string& name)
{
  std::string clean = name;

  size_t pos = clean.find("//");
  while (pos != std::string::npos)
  {
    clean.erase(pos, 1);
    pos = clean.find("//", pos);
  }

  if (*clean.rbegin() == '/')
  {
    clean.erase(clean.size() - 1, 1);
  }

  return clean;
}

std::string append(const std::string& left, const std::string& right)
{
  return clean(left + "/" + right);
}

namespace map_merge
{
struct MapSubscription {
  // protects consistency of writable_map and readonly_map
  // also protects reads and writes of shared_ptrs
  std::mutex mutex;

  geometry_msgs::msg::Transform initial_pose;
  nav_msgs::msg::OccupancyGrid::Ptr writable_map;
  nav_msgs::msg::OccupancyGrid::ConstPtr readonly_map;

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

  rclcpp::Logger logger_ = rclcpp::get_logger("MapMergeNode");

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

  void spin();
  void executetopicSubscribing();
  void executemapMerging();
  void executeposeEstimation();

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
