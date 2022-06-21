/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, Jiri Horner.
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

#include <combine_grids/grid_warper.h>
#include <gtest/gtest.h>
// #include <ros/console.h>
#include <opencv2/core/utility.hpp>
#include "testing_helpers.h"

#define private public
#include <combine_grids/merging_pipeline.h>

const std::array<const char*, 2> hector_maps = {
    "map00.pgm",
    "map05.pgm",
};

const std::array<const char*, 2> gmapping_maps = {
    "2011-08-09-12-22-52.pgm",
    "2012-01-28-11-12-01.pgm",
};

constexpr bool verbose_tests = false;

#define EXPECT_VALID_GRID(grid)                                                \
  ASSERT_TRUE(static_cast<bool>(grid));                                        \
  EXPECT_TRUE(consistentData(*grid));                                          \
  EXPECT_GT(grid->info.resolution, 0);                                         \
  EXPECT_TRUE(isIdentity(grid->info.origin.orientation))

TEST(MergingPipeline, canStich0Grid)
{
  std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> maps;
  combine_grids::MergingPipeline merger;
  merger.feed(maps.begin(), maps.end());
  EXPECT_TRUE(merger.estimateTransforms());
  EXPECT_EQ(merger.composeGrids(), nullptr);
  EXPECT_EQ(merger.getTransforms().size(), (long unsigned int) 0);
}

TEST(MergingPipeline, canStich1Grid)
{
  auto map = loadMap(hector_maps[1]);
  combine_grids::MergingPipeline merger;
  merger.feed(&map, &map + 1);
  merger.estimateTransforms();
  auto merged_grid = merger.composeGrids();

  EXPECT_VALID_GRID(merged_grid);
  // don't use EXPECT_EQ, since it prints too much info
  EXPECT_TRUE(maps_equal(*merged_grid, *map));

  // check estimated transforms
  auto transforms = merger.getTransforms();
  EXPECT_EQ(transforms.size(), (long unsigned int) 1);
  EXPECT_TRUE(isIdentity(transforms[0]));
}

TEST(MergingPipeline, canStich2Grids)
{
  auto maps = loadMaps(hector_maps.begin(), hector_maps.end());
  combine_grids::MergingPipeline merger;
  merger.feed(maps.begin(), maps.end());
  merger.estimateTransforms();
  auto merged_grid = merger.composeGrids();

  EXPECT_VALID_GRID(merged_grid);
  // grid size should indicate sucessful merge
  EXPECT_NEAR(2091, merged_grid->info.width, 30);
  EXPECT_NEAR(2091, merged_grid->info.height, 30);

  if (verbose_tests) {
    saveMap("test_canStich2Grids.pgm", merged_grid);
  }
}

TEST(MergingPipeline, canStichGridsGmapping)
{
  auto maps = loadMaps(gmapping_maps.begin(), gmapping_maps.end());
  combine_grids::MergingPipeline merger;
  merger.feed(maps.begin(), maps.end());
  merger.estimateTransforms();
  auto merged_grid = merger.composeGrids();

  EXPECT_VALID_GRID(merged_grid);
  // grid size should indicate sucessful merge
  EXPECT_NEAR(5427, merged_grid->info.width, 30);
  EXPECT_NEAR(5427, merged_grid->info.height, 30);

  if (verbose_tests) {
    saveMap("canStichGridsGmapping.pgm", merged_grid);
  }
}

TEST(MergingPipeline, estimationAccuracy)
{
  // for this test we measure estimation on the same grid artificially
  // transformed
  double angle = 0.523599 /* 30 deg in rads*/;
  // TODO test also translations
  double tx = 0;
  double ty = 0;
  cv::Matx23d transform{std::cos(angle), -std::sin(angle), tx,
                        std::sin(angle), std::cos(angle),  ty};

  auto map = loadMap(hector_maps[1]);
  combine_grids::MergingPipeline merger;
  merger.feed(&map, &map + 1);

  // warp the map with Affine Transform
  combine_grids::internal::GridWarper warper;
  cv::Mat warped;
  auto roi = warper.warp(merger.images_[0], cv::Mat(transform), warped);

  // add warped map
  // this relies on internal implementation of merging pipeline
  merger.grids_.emplace_back();
  merger.images_.push_back(warped);

  merger.estimateTransforms();
  auto merged_grid = merger.composeGrids();

  EXPECT_VALID_GRID(merged_grid);
  // transforms
  auto transforms = merger.getTransforms();
  EXPECT_EQ(transforms.size(), (long unsigned int) 2);
  EXPECT_TRUE(isIdentity(transforms[0]));
  tf2::Transform t;
  tf2::fromMsg(transforms[1], t);

  EXPECT_NEAR(angle, t.getRotation().getAngle(), 1e-2);
  EXPECT_NEAR(tx - roi.tl().x, t.getOrigin().x(), 2);
  EXPECT_NEAR(ty - roi.tl().y, t.getOrigin().y(), 2);
}

TEST(MergingPipeline, transformsRoundTrip)
{
  auto map = loadMap(hector_maps[0]);
  combine_grids::MergingPipeline merger;
  merger.feed(&map, &map + 1);
  for (size_t i = 0; i < 1000; ++i) {
    auto in_transform = randomTransform();
    merger.setTransforms(&in_transform, &in_transform + 1);

    auto out_transforms = merger.getTransforms();
    ASSERT_EQ(out_transforms.size(), (long unsigned int) 1);
    auto out_transform = out_transforms[0];
    EXPECT_FLOAT_EQ(in_transform.translation.x, out_transform.translation.x);
    EXPECT_FLOAT_EQ(in_transform.translation.y, out_transform.translation.y);
    EXPECT_FLOAT_EQ(in_transform.translation.z, out_transform.translation.z);
    EXPECT_FLOAT_EQ(in_transform.rotation.x, out_transform.rotation.x);
    EXPECT_FLOAT_EQ(in_transform.rotation.y, out_transform.rotation.y);
    EXPECT_FLOAT_EQ(in_transform.rotation.z, out_transform.rotation.z);
    EXPECT_FLOAT_EQ(in_transform.rotation.w, out_transform.rotation.w);
  }
}

TEST(MergingPipeline, setTransformsInternal)
{
  auto map = loadMap(hector_maps[0]);
  combine_grids::MergingPipeline merger;
  merger.feed(&map, &map + 1);

  for (size_t i = 0; i < 1000; ++i) {
    auto transform = randomTransform();
    merger.setTransforms(&transform, &transform + 1);

    ASSERT_EQ(merger.transforms_.size(), (long unsigned int) 1);
    auto& transform_internal = merger.transforms_[0];
    // verify that transforms are the same in 2D
    tf2::Vector3 a[2] = {{1., 0., 1.}, {0., 1., 1.}};
    cv::Point3d b[2] = {{1., 0., 1.}, {0., 1., 1.}};
    for (auto j : {0, 1}) {
      tf2::Transform t;
      fromMsg(transform, t);
      auto p1 = t * a[j];
      cv::Mat p2 = transform_internal * cv::Mat(b[j]);
      // some precision is naturally lost during conversion, float precision is
      // still good for us
      EXPECT_FLOAT_EQ(p1.x(), p2.at<double>(0));
      EXPECT_FLOAT_EQ(p1.y(), p2.at<double>(1));
    }
  }
}

TEST(MergingPipeline, getTransformsInternal)
{
  auto map = loadMap(hector_maps[0]);
  combine_grids::MergingPipeline merger;
  merger.feed(&map, &map + 1);

  // set internal transform
  merger.transforms_.resize(1);
  for (size_t i = 0; i < 1000; ++i) {
    cv::Mat transform_internal = randomTransformMatrix();
    merger.transforms_[0] = transform_internal;
    auto transforms = merger.getTransforms();
    ASSERT_EQ(transforms.size(), (long unsigned int) 1);
    // output quaternion should be normalized
    auto& q = transforms[0].rotation;
    EXPECT_DOUBLE_EQ(1., q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

    // verify that transforms are the same in 2D
    tf2::Transform transform;
    fromMsg(transforms[0], transform);
    tf2::Vector3 a[2] = {{1., 0., 1.}, {0., 1., 1.}};
    cv::Point3d b[2] = {{1., 0., 1.}, {0., 1., 1.}};
    for (auto j : {0, 1}) {
      auto p1 = transform * a[j];
      cv::Mat p2 = transform_internal * cv::Mat(b[j]);
      EXPECT_FLOAT_EQ(p1.x(), p2.at<double>(0));
      EXPECT_FLOAT_EQ(p1.y(), p2.at<double>(1));
    }
  }
}

TEST(MergingPipeline, setEmptyTransforms)
{
  constexpr size_t size = 2;
  std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> maps(size);
  std::vector<geometry_msgs::msg::Transform> transforms(size);
  combine_grids::MergingPipeline merger;
  merger.feed(maps.begin(), maps.end());
  merger.setTransforms(transforms.begin(), transforms.end());
  EXPECT_EQ(merger.composeGrids(), nullptr);
  EXPECT_EQ(merger.getTransforms().size(), size);
}

/* empty image may end with identity transform. */
TEST(MergingPipeline, emptyImageWithTransform)
{
  constexpr size_t size = 1;
  std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> maps(size);
  std::vector<geometry_msgs::msg::Transform> transforms(size);
  transforms[0].rotation.z = 1;  // set transform to identity
  combine_grids::MergingPipeline merger;
  merger.feed(maps.begin(), maps.end());
  merger.setTransforms(transforms.begin(), transforms.end());
  EXPECT_EQ(merger.composeGrids(), nullptr);
  EXPECT_EQ(merger.getTransforms().size(), size);
}

/* one image may be empty */
TEST(MergingPipeline, oneEmptyImage)
{
  std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> maps{nullptr,
                                                    loadMap(gmapping_maps[0])};
  combine_grids::MergingPipeline merger;
  merger.feed(maps.begin(), maps.end());
  merger.estimateTransforms();
  auto merged_grid = merger.composeGrids();
  auto transforms = merger.getTransforms();

  EXPECT_VALID_GRID(merged_grid);
  // don't use EXPECT_EQ, since it prints too much info
  EXPECT_TRUE(maps_equal(*merged_grid, *maps[1]));
  // transforms
  EXPECT_EQ(transforms.size(), (long unsigned int) 2);
  EXPECT_TRUE(isIdentity(transforms[1]));
}

// non-identity known positions etc.
TEST(MergingPipeline, knownInitPositions)
{
  auto maps = loadMaps(gmapping_maps.begin(), gmapping_maps.end());
  combine_grids::MergingPipeline merger;
  merger.feed(maps.begin(), maps.end());

  for (size_t i = 0; i < 5; ++i) {
    std::vector<geometry_msgs::msg::Transform> transforms{randomTransform(),
                                                     randomTransform()};
    merger.setTransforms(transforms.begin(), transforms.end());
    auto merged_grid = merger.composeGrids();

    EXPECT_VALID_GRID(merged_grid);
  }
}

int main(int argc, char** argv)
{
  // ros::Time::init();
  // if (verbose_tests &&
  //     ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //                                    ros::console::levels::Debug)) {
  //   ros::console::notifyLoggerLevelsChanged();
  // }
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
