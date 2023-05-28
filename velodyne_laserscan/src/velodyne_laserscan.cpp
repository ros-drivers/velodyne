// Copyright 2018, 2019 Kevin Hallenbeck, Joshua Whitley
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

#include "velodyne_laserscan/velodyne_laserscan.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <utility>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace velodyne_laserscan
{

VelodyneLaserScan::VelodyneLaserScan(const rclcpp::NodeOptions & options)
: rclcpp::Node("velodyne_laserscan_node", options)
{
  rcl_interfaces::msg::ParameterDescriptor ring_desc;
  ring_desc.name = "ring";
  ring_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  ring_desc.description = "Ring";
  rcl_interfaces::msg::IntegerRange ring_range;
  ring_range.from_value = -1;
  ring_range.to_value = 31;
  ring_desc.integer_range.push_back(ring_range);
  ring_ = declare_parameter("ring", -1, ring_desc);

  rcl_interfaces::msg::ParameterDescriptor resolution_desc;
  resolution_desc.name = "resolution";
  resolution_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  resolution_desc.description = "Resolution";
  rcl_interfaces::msg::FloatingPointRange resolution_range;
  resolution_range.from_value = 0.001;
  resolution_range.to_value = 0.05;
  resolution_desc.floating_point_range.push_back(resolution_range);
  resolution_ = declare_parameter("resolution", 0.007, resolution_desc);

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "velodyne_points", rclcpp::QoS(10),
    std::bind(&VelodyneLaserScan::recvCallback, this, std::placeholders::_1));
  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
}

void VelodyneLaserScan::recvCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Latch ring count
  if (!ring_count_) {
    // Check for PointCloud2 field 'ring'
    bool found = false;
    for (size_t i = 0; i < msg->fields.size(); i++) {
      if (msg->fields[i].datatype == sensor_msgs::msg::PointField::UINT16) {
        if (msg->fields[i].name == "ring") {
          found = true;
          break;
        }
      }
    }

    if (!found) {
      RCLCPP_ERROR(this->get_logger(), "Field 'ring' of type 'UINT16' not present in PointCloud2");
      return;
    }

    for (sensor_msgs::PointCloud2ConstIterator<uint16_t> it(*msg, "ring"); it != it.end(); ++it) {
      const uint16_t ring = *it;

      if (ring + 1 > ring_count_) {
        ring_count_ = ring + 1;
      }
    }

    if (ring_count_) {
      RCLCPP_INFO(this->get_logger(), "Latched ring count of %u", ring_count_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Field 'ring' of type 'UINT16' not present in PointCloud2");
      return;
    }
  }

  // Select ring to use
  uint16_t ring;

  if ((ring_ < 0) || (ring_ >= ring_count_)) {
    // Default to ring closest to being level for each known sensor
    if (ring_count_ > 32) {
      ring = 57;  // HDL-64E
    } else if (ring_count_ > 16) {
      ring = 23;  // HDL-32E
    } else {
      ring = 8;  // VLP-16
    }
  } else {
    ring = ring_;
  }

  // Load structure of PointCloud2
  int offset_x = -1;
  int offset_y = -1;
  int offset_z = -1;
  int offset_i = -1;
  int offset_r = -1;

  for (size_t i = 0; i < msg->fields.size(); i++) {
    if (msg->fields[i].datatype == sensor_msgs::msg::PointField::FLOAT32) {
      if (msg->fields[i].name == "x") {
        offset_x = msg->fields[i].offset;
      } else if (msg->fields[i].name == "y") {
        offset_y = msg->fields[i].offset;
      } else if (msg->fields[i].name == "z") {
        offset_z = msg->fields[i].offset;
      } else if (msg->fields[i].name == "intensity") {
        offset_i = msg->fields[i].offset;
      }
    } else if (msg->fields[i].datatype == sensor_msgs::msg::PointField::UINT16) {
      if (msg->fields[i].name == "ring") {
        offset_r = msg->fields[i].offset;
      }
    }
  }

  (void)offset_z;

  // Construct LaserScan message
  if ((offset_x >= 0) && (offset_y >= 0) && (offset_r >= 0)) {
    const float kResolution = std::abs(resolution_);
    const size_t kSize = std::round(2.0 * M_PI / kResolution);
    auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan->header = msg->header;
    scan->angle_increment = kResolution;
    scan->angle_min = -M_PI;
    scan->angle_max = M_PI;
    scan->range_min = 0.0;
    scan->range_max = 200.0;
    scan->time_increment = 0.0;
    scan->ranges.resize(kSize, INFINITY);

    if ((offset_x == 0) &&
      (offset_y == 4) &&
      (offset_i % 4 == 0) &&
      (offset_r % 4 == 0))
    {
      scan->intensities.resize(kSize);

      const size_t X = 0;
      const size_t Y = 1;
      const size_t I = offset_i / 4;
      const size_t R = offset_r / 4;
      for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
        // Field "ring" is of UINT16 type, 2 bytes long. But this loop's iterator assumes FLOAT32
        // type fields, 4 bytes long. Thus, de-referencing it (even) at the right offset will
        // only yield "ring" field bytes, plus 2 bytes right after it, interpreted as a float
        // value. We can, however, re-interpret that float value binary representation as that
        // of an unsigned integer, 16 bit long.
        const uint16_t r = *(reinterpret_cast<const uint16_t *>(&it[R]));

        if (r == ring) {
          const float x = it[X];  // x
          const float y = it[Y];  // y
          const float i = it[I];  // intensity
          const int bin = (::atan2f(y, x) + static_cast<float>(M_PI)) / kResolution;

          if ((bin >= 0) && (bin < static_cast<int>(kSize))) {
            scan->ranges[bin] = ::sqrtf(x * x + y * y);
            scan->intensities[bin] = i;
          }
        }
      }
    } else {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "PointCloud2 fields in unexpected order. Using slower generic method.");

      if (offset_i >= 0) {
        scan->intensities.resize(kSize);
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity");

        for (; iter_r != iter_r.end(); ++iter_x, ++iter_y, ++iter_r, ++iter_i) {
          const uint16_t r = *iter_r;  // ring

          if (r == ring) {
            const float x = *iter_x;  // x
            const float y = *iter_y;  // y
            const float i = *iter_i;  // intensity
            const int bin = (::atan2f(y, x) + static_cast<float>(M_PI)) / kResolution;

            if ((bin >= 0) && (bin < static_cast<int>(kSize))) {
              scan->ranges[bin] = ::sqrtf(x * x + y * y);
              scan->intensities[bin] = i;
            }
          }
        }
      } else {
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");

        for (; iter_r != iter_r.end(); ++iter_x, ++iter_y, ++iter_r) {
          const uint16_t r = *iter_r;  // ring

          if (r == ring) {
            const float x = *iter_x;  // x
            const float y = *iter_y;  // y
            const int bin = (::atan2f(y, x) + static_cast<float>(M_PI)) / kResolution;

            if ((bin >= 0) && (bin < static_cast<int>(kSize))) {
              scan->ranges[bin] = ::sqrtf(x * x + y * y);
            }
          }
        }
      }
    }

    pub_->publish(std::move(scan));
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointCloud2 missing one or more required fields! (x,y,ring)");
  }
}

}  // namespace velodyne_laserscan

RCLCPP_COMPONENTS_REGISTER_NODE(velodyne_laserscan::VelodyneLaserScan)
