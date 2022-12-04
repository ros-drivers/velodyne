// Copyright 2019 Open Source Robotics Foundation
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <tf2/buffer_core.h>

#include <memory>

#include "velodyne_pointcloud/datacontainerbase.hpp"

class TestContainer final
  : public velodyne_rawdata::DataContainerBase
{
public:
  TestContainer(unsigned int width, rclcpp::Clock::SharedPtr clock)
  : velodyne_rawdata::DataContainerBase(
      0, 0, "target", "fixed", width, 0, false, 0,
      clock, 1, "x", 1, sensor_msgs::msg::PointField::FLOAT32)
  {
  }

  void addPoint(
    float x, float y, float z, const uint16_t ring,
    const float distance, const float intensity, const float time) override
  {
    (void)x;
    (void)y;
    (void)z;
    (void)ring;
    (void)distance;
    (void)intensity;
    (void)time;
  }

  void newLine() override
  {
  }

  uint32_t getCloudWidth()
  {
    return cloud.width;
  }

  uint32_t getCloudPointStep()
  {
    return cloud.point_step;
  }

  uint32_t getCloudRowStep()
  {
    return cloud.row_step;
  }
};

TEST(datacontainerbase, row_step_zero_width_constructor)
{
  auto clock = std::make_shared<rclcpp::Clock>();
  TestContainer cont(0, clock);
  ASSERT_EQ(cont.getCloudWidth(), 0U);
  ASSERT_EQ(cont.getCloudPointStep(), 4U);
  ASSERT_EQ(cont.getCloudRowStep(), 0U);
}

TEST(datacontainerbase, row_step_one_width_constructor)
{
  auto clock = std::make_shared<rclcpp::Clock>();
  TestContainer cont(1, clock);
  ASSERT_EQ(cont.getCloudWidth(), 1U);
  ASSERT_EQ(cont.getCloudPointStep(), 4U);
  ASSERT_EQ(cont.getCloudRowStep(), 4U);
}

TEST(datacontainerbase, row_step_one_width_after_setup)
{
  auto clock = std::make_shared<rclcpp::Clock>();
  TestContainer cont(1, clock);
  auto msg = std::make_shared<velodyne_msgs::msg::VelodyneScan>();
  cont.setup(msg);
  ASSERT_EQ(cont.getCloudWidth(), 1U);
  ASSERT_EQ(cont.getCloudPointStep(), 4U);
  ASSERT_EQ(cont.getCloudRowStep(), 4U);
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
