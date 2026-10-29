/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Carlos Alvarez.
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
 *   * Neither the name of the Carlos Alvarez nor the names of its
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

#include <gtest/gtest.h>
#include <explore/frontier_search.h>
#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <cmath>

#define private public

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

TEST(Explore, testSameGoal)
{
    geometry_msgs::msg::Point goal1;
    geometry_msgs::msg::Point goal2;
    // Populate the goal with known values
    goal1.x = 1.0;
    goal1.y = 2.0;
    goal1.z = 3.0;

    goal2.x = 0.0;
    goal2.y = 0.0;
    goal2.z = 0.0;
    auto same_goal = same_point(goal1, goal2);
    EXPECT_FALSE(same_goal);
    goal2.x = goal1.x;
    goal2.y = goal1.y;
    goal2.z = goal1.z;
    same_goal = same_point(goal1, goal2);
    EXPECT_TRUE(same_goal);
}

// Searches a free room of cells [100, 139] x [80, 120], walled with lethal
// cells except for cells [gap_start, gap_end] of the wall at x = 140. Those
// stay unknown, so they are the only frontier. The robot is at cell
// (120, 100) and the map origin at (0, 0), 5-7 m away, where a centroid that
// leaves out a cell is visibly off.
static std::vector<frontier_exploration::Frontier>
searchRoomWithGap(unsigned int gap_start, unsigned int gap_end)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.05, 0.0, 0.0,
                                 nav2_costmap_2d::NO_INFORMATION);
  for (unsigned int x = 99; x <= 140; ++x) {
    for (unsigned int y = 79; y <= 121; ++y) {
      bool wall = x == 99 || x == 140 || y == 79 || y == 121;
      bool gap = x == 140 && y >= gap_start && y <= gap_end;
      if (!gap) {
        map.setCost(x, y,
                    wall ? nav2_costmap_2d::LETHAL_OBSTACLE :
                           nav2_costmap_2d::FREE_SPACE);
      }
    }
  }

  frontier_exploration::FrontierSearch search(&map, 1.0, 1.0, 0.0,
                                              rclcpp::get_logger("test"));
  geometry_msgs::msg::Point robot;
  map.mapToWorld(120, 100, robot.x, robot.y);
  return search.searchFrom(robot);
}

TEST(FrontierSearch, testFrontierIncludesEveryCell)
{
  // centre of map cell i, in metres
  auto world = [](double i) { return (i + 0.5) * 0.05; };

  // {gap_start, gap_end}: the gap's middle cell, a single cell, and a gap
  // whose mean lies between two cells
  for (auto gap : {std::make_pair(93u, 107u), std::make_pair(100u, 100u),
                   std::make_pair(100u, 109u)}) {
    SCOPED_TRACE("gap of cells " + std::to_string(gap.first) + "-" +
                 std::to_string(gap.second));
    auto frontiers = searchRoomWithGap(gap.first, gap.second);
    ASSERT_EQ(frontiers.size(), 1u);
    const auto& frontier = frontiers[0];

    unsigned int cells = gap.second - gap.first + 1;
    EXPECT_EQ(frontier.size, cells);
    EXPECT_EQ(frontier.points.size(), cells);
    // the centroid is the mean of the gap's cells
    EXPECT_NEAR(frontier.centroid.x, world(140), 1e-9);
    EXPECT_NEAR(frontier.centroid.y, world((gap.first + gap.second) / 2.0),
                1e-9);
    // the gap cell closest to the robot is (140, 100), 20 cells away
    EXPECT_NEAR(frontier.min_distance, 20 * 0.05, 1e-9);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
