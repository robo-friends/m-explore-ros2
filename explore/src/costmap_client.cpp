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

#include <explore/costmap_client.h>
#include <unistd.h>

#include <functional>
#include <mutex>
#include <string>

namespace explore
{
// static translation table to speed things up
std::array<unsigned char, 256> init_translation_table();
static const std::array<unsigned char, 256> cost_translation_table__ =
    init_translation_table();

Costmap2DClient::Costmap2DClient(rclcpp::Node& node, const tf2_ros::Buffer* tf)
  : tf_(tf), node_(node)
{
  std::string costmap_topic;
  std::string costmap_updates_topic;

  node_.declare_parameter<std::string>("costmap_topic", std::string("costmap"));
  node_.declare_parameter<std::string>("costmap_updates_topic",
                                       std::string("costmap_updates"));
  node_.declare_parameter<std::string>("robot_base_frame", std::string("base_"
                                                                       "link"));
  // transform tolerance is used for all tf transforms here
  node_.declare_parameter<double>("transform_tolerance", 0.3);

  node_.get_parameter("costmap_topic", costmap_topic);
  node_.get_parameter("costmap_updates_topic", costmap_updates_topic);
  node_.get_parameter("robot_base_frame", robot_base_frame_);
  node_.get_parameter("transform_tolerance", transform_tolerance_);

  /* initialize costmap */
  costmap_sub_ = node_.create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic, 1000,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        costmap_received_ = true;
        updateFullMap(msg);
      });

  // ROS1 CODE
  // auto costmap_msg =
  // ros::topic::waitForMessage<nav_msgs::msg::OccupancyGrid>(
  //     costmap_topic, subscription_nh);

  // Spin some until the callback gets called to replicate
  // ros::topic::waitForMessage
  RCLCPP_INFO(node_.get_logger(),
              "Waiting for costmap to become available, topic: %s",
              costmap_topic.c_str());
  while (!costmap_received_) {
    rclcpp::spin_some(node_.get_node_base_interface());
    // Wait for a second
    usleep(1000000);
  }
  // updateFullMap(costmap_msg); // this is already called in the callback of
  // the costmap_sub_

  /* subscribe to map updates */
  costmap_updates_sub_ =
      node_.create_subscription<map_msgs::msg::OccupancyGridUpdate>(
          costmap_updates_topic, 1000,
          [this](const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
            updatePartialMap(msg);
          });

  // ROS1 CODE.
  // TODO: Do we need this?
  /* resolve tf prefix for robot_base_frame */
  // std::string tf_prefix = tf::getPrefixParam(node_);
  // robot_base_frame_ = tf::resolve(tf_prefix, robot_base_frame_);

  // we need to make sure that the transform between the robot base frame and
  // the global frame is available

  // the global frame is set in the costmap callback. This is why we need to
  // ensure that a costmap is received

  /* tf transform is necessary for getRobotPose */
  auto last_error = node_.now();
  std::string tf_error;
  while (rclcpp::ok() &&
         !tf_->canTransform(global_frame_, robot_base_frame_,
                            tf2::TimePointZero, tf2::durationFromSec(0.1),
                            &tf_error)) {
    rclcpp::spin_some(node_.get_node_base_interface());
    if (last_error + tf2::durationFromSec(5.0) < node_.now()) {
      RCLCPP_WARN(node_.get_logger(),
                  "Timed out waiting for transform from %s to %s to become "
                  "available "
                  "before subscribing to costmap, tf error: %s",
                  robot_base_frame_.c_str(), global_frame_.c_str(),
                  tf_error.c_str());
      last_error = node_.now();
      ;
    }
    // The error string will accumulate and errors will typically be the same,
    // so the last
    // will do for the warning above. Reset the string here to avoid
    // accumulation.
    tf_error.clear();
  }
}

void Costmap2DClient::updateFullMap(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  global_frame_ = msg->header.frame_id;

  unsigned int size_in_cells_x = msg->info.width;
  unsigned int size_in_cells_y = msg->info.height;
  double resolution = msg->info.resolution;
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;

  RCLCPP_DEBUG(node_.get_logger(), "received full new map, resizing to: %d, %d",
               size_in_cells_x, size_in_cells_y);
  costmap_.resizeMap(size_in_cells_x, size_in_cells_y, resolution, origin_x,
                     origin_y);

  // lock as we are accessing raw underlying map
  auto* mutex = costmap_.getMutex();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

  // fill map with data
  unsigned char* costmap_data = costmap_.getCharMap();
  size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
  RCLCPP_DEBUG(node_.get_logger(), "full map update, %lu values", costmap_size);
  for (size_t i = 0; i < costmap_size && i < msg->data.size(); ++i) {
    unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
    costmap_data[i] = cost_translation_table__[cell_cost];
  }
  RCLCPP_DEBUG(node_.get_logger(), "map updated, written %lu values",
               costmap_size);
}

void Costmap2DClient::updatePartialMap(
    const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
{
  RCLCPP_DEBUG(node_.get_logger(), "received partial map update");
  global_frame_ = msg->header.frame_id;

  if (msg->x < 0 || msg->y < 0) {
    RCLCPP_DEBUG(node_.get_logger(),
                 "negative coordinates, invalid update. x: %d, y: %d", msg->x,
                 msg->y);
    return;
  }

  size_t x0 = static_cast<size_t>(msg->x);
  size_t y0 = static_cast<size_t>(msg->y);
  size_t xn = msg->width + x0;
  size_t yn = msg->height + y0;

  // lock as we are accessing raw underlying map
  auto* mutex = costmap_.getMutex();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

  size_t costmap_xn = costmap_.getSizeInCellsX();
  size_t costmap_yn = costmap_.getSizeInCellsY();

  if (xn > costmap_xn || x0 > costmap_xn || yn > costmap_yn ||
      y0 > costmap_yn) {
    RCLCPP_WARN(node_.get_logger(),
                "received update doesn't fully fit into existing map, "
                "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
                "map is: [0, %lu], [0, %lu]",
                x0, xn, y0, yn, costmap_xn, costmap_yn);
  }

  // update map with data
  unsigned char* costmap_data = costmap_.getCharMap();
  size_t i = 0;
  for (size_t y = y0; y < yn && y < costmap_yn; ++y) {
    for (size_t x = x0; x < xn && x < costmap_xn; ++x) {
      size_t idx = costmap_.getIndex(x, y);
      unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
      costmap_data[idx] = cost_translation_table__[cell_cost];
      ++i;
    }
  }
}

geometry_msgs::msg::Pose Costmap2DClient::getRobotPose() const
{
  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Pose empty_pose;
  robot_pose.header.frame_id = robot_base_frame_;
  robot_pose.header.stamp = node_.now();

  auto& clk = *node_.get_clock();

  // get the global pose of the robot
  try {
    robot_pose = tf_->transform(robot_pose, global_frame_,
                                tf2::durationFromSec(transform_tolerance_));
  } catch (tf2::LookupException& ex) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), clk, 1000,
                          "No Transform available Error looking up robot pose: "
                          "%s\n",
                          ex.what());
    return empty_pose;
  } catch (tf2::ConnectivityException& ex) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), clk, 1000,
                          "Connectivity Error looking up robot pose: %s\n",
                          ex.what());
    return empty_pose;
  } catch (tf2::ExtrapolationException& ex) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), clk, 1000,
                          "Extrapolation Error looking up robot pose: %s\n",
                          ex.what());
    return empty_pose;
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), clk, 1000, "Other error: %s\n",
                          ex.what());
    return empty_pose;
  }

  return robot_pose.pose;
}

std::array<unsigned char, 256> init_translation_table()
{
  std::array<unsigned char, 256> cost_translation_table;

  // lineary mapped from [0..100] to [0..255]
  for (size_t i = 0; i < 256; ++i) {
    cost_translation_table[i] =
        static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
  }

  // special values:
  cost_translation_table[0] = 0;      // NO obstacle
  cost_translation_table[99] = 253;   // INSCRIBED obstacle
  cost_translation_table[100] = 254;  // LETHAL obstacle
  cost_translation_table[static_cast<unsigned char>(-1)] = 255;  // UNKNOWN

  return cost_translation_table;
}

}  // namespace explore
