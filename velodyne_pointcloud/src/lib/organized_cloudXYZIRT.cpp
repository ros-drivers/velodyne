// Copyright 2019 Sebastian Pütz, Joshua Whitley
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

#include <tf2/buffer_core.h>

#include <cmath>
#include <memory>
#include <string>

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "velodyne_pointcloud/organized_cloudXYZIRT.hpp"

namespace velodyne_pointcloud
{

OrganizedCloudXYZIRT::OrganizedCloudXYZIRT(
  const double min_range, const double max_range,
  const std::string & target_frame, const std::string & fixed_frame,
  const unsigned int num_lasers, const unsigned int scans_per_block,
  rclcpp::Clock::SharedPtr clock)
: DataContainerBase(
    min_range, max_range, target_frame, fixed_frame,
    num_lasers, 0, false, scans_per_block, clock, 6,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "ring", 1, sensor_msgs::msg::PointField::UINT16,
    "time", 1, sensor_msgs::msg::PointField::FLOAT32),
  iter_x_(cloud, "x"), iter_y_(cloud, "y"), iter_z_(cloud, "z"),
  iter_intensity_(cloud, "intensity"), iter_ring_(cloud, "ring"), iter_time_(cloud, "time")
{
}

void OrganizedCloudXYZIRT::newLine()
{
  iter_x_ = iter_x_ + config_.init_width;
  iter_y_ = iter_y_ + config_.init_width;
  iter_z_ = iter_z_ + config_.init_width;
  iter_ring_ = iter_ring_ + config_.init_width;
  iter_intensity_ = iter_intensity_ + config_.init_width;
  iter_time_ = iter_time_ + config_.init_width;
  ++cloud.height;
}

void OrganizedCloudXYZIRT::setup(const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr scan_msg)
{
  DataContainerBase::setup(scan_msg);
  iter_x_ = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
  iter_y_ = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
  iter_z_ = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
  iter_intensity_ = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
  iter_ring_ = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "ring");
  iter_time_ = sensor_msgs::PointCloud2Iterator<float>(cloud, "time");
}

void OrganizedCloudXYZIRT::addPoint(
  float x, float y, float z, const uint16_t ring,
  const float distance, const float intensity, const float time)
{
  /** The laser values are not ordered, the organized structure
   * needs ordered neighbour points. The right order is defined
   * by the laser_ring value.
   * To keep the right ordering, the filtered values are set to
   * NaN.
   */
  if (pointInRange(distance)) {
    transformPoint(x, y, z);

    *(iter_x_ + ring) = x;
    *(iter_y_ + ring) = y;
    *(iter_z_ + ring) = z;
    *(iter_intensity_ + ring) = intensity;
    *(iter_ring_ + ring) = ring;
    *(iter_time_ + ring) = time;
  } else {
    *(iter_x_ + ring) = nanf("");
    *(iter_y_ + ring) = nanf("");
    *(iter_z_ + ring) = nanf("");
    *(iter_intensity_ + ring) = ::nanf("");
    *(iter_ring_ + ring) = ring;
    *(iter_time_ + ring) = time;
  }
}

}  // namespace velodyne_pointcloud
