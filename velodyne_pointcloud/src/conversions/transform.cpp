// Copyright 2009, 2010, 2011, 2012, 2019 Austin Robot Technology, Jack O'Quin, Jesse Vera, Sebastian Pütz, Joshua Whitley  // NOLINT
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

#include "velodyne_pointcloud/transform.hpp"

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "velodyne_pointcloud/organized_cloudXYZIRT.hpp"
#include "velodyne_pointcloud/pointcloudXYZIRT.hpp"
#include "velodyne_pointcloud/rawdata.hpp"

namespace velodyne_pointcloud
{

Transform::Transform(const rclcpp::NodeOptions & options)
: rclcpp::Node("velodyne_transform_node", options),
  diagnostics_(this)
{
  std::string calibration_file = this->declare_parameter("calibration", "");
  const auto model = this->declare_parameter("model", "64E");

  rcl_interfaces::msg::ParameterDescriptor min_range_desc;
  min_range_desc.name = "min_range";
  min_range_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  min_range_desc.description = "minimum range to publish";
  rcl_interfaces::msg::FloatingPointRange min_range_range;
  min_range_range.from_value = 0.1;
  min_range_range.to_value = 10.0;
  min_range_desc.floating_point_range.push_back(min_range_range);
  double min_range = this->declare_parameter("min_range", 0.9, min_range_desc);

  rcl_interfaces::msg::ParameterDescriptor max_range_desc;
  max_range_desc.name = "max_range";
  max_range_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  max_range_desc.description = "maximum range to publish";
  rcl_interfaces::msg::FloatingPointRange max_range_range;
  max_range_range.from_value = 0.1;
  max_range_range.to_value = 300.0;
  max_range_desc.floating_point_range.push_back(max_range_range);
  double max_range = this->declare_parameter("max_range", 130.0, max_range_desc);

  rcl_interfaces::msg::ParameterDescriptor view_direction_desc;
  view_direction_desc.name = "view_direction";
  view_direction_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  view_direction_desc.description = "angle defining the center of view";
  rcl_interfaces::msg::FloatingPointRange view_direction_range;
  view_direction_range.from_value = -M_PI;
  view_direction_range.to_value = M_PI;
  view_direction_desc.floating_point_range.push_back(view_direction_range);
  double view_direction = this->declare_parameter("view_direction", 0.0, view_direction_desc);

  rcl_interfaces::msg::ParameterDescriptor view_width_desc;
  view_width_desc.name = "view_width";
  view_width_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  view_width_desc.description = "angle defining the view width";
  rcl_interfaces::msg::FloatingPointRange view_width_range;
  view_width_range.from_value = 0.0;
  view_width_range.to_value = 2.0 * M_PI;
  view_width_desc.floating_point_range.push_back(view_width_range);
  double view_width = this->declare_parameter("view_width", 2.0 * M_PI, view_width_desc);

  std::string fixed_frame = this->declare_parameter("fixed_frame", "");
  std::string target_frame = this->declare_parameter("target_frame", "");
  bool organize_cloud = this->declare_parameter("organize_cloud", true);

  RCLCPP_INFO(this->get_logger(), "correction angles: %s", calibration_file.c_str());

  data_ = std::make_unique<velodyne_rawdata::RawData>(calibration_file, model);

  if (organize_cloud) {
    container_ptr_ = std::make_unique<OrganizedCloudXYZIRT>(
      min_range, max_range, target_frame, fixed_frame, data_->numLasers(),
      data_->scansPerPacket(), this->get_clock());
  } else {
    container_ptr_ = std::make_unique<PointcloudXYZIRT>(
      min_range, max_range, target_frame, fixed_frame,
      data_->scansPerPacket(), this->get_clock());
  }

  // advertise output point cloud (before subscribing to input data)
  output_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points", 10);

  velodyne_scan_ = this->create_subscription<velodyne_msgs::msg::VelodyneScan>(
    "velodyne_packets", 10, std::bind(&Transform::processScan, this, std::placeholders::_1));

  // Diagnostics
  diagnostics_.setHardwareID("Velodyne Transform");
  // Arbitrary frequencies since we don't know which RPM is used, and are only
  // concerned about monitoring the frequency.
  diag_min_freq_ = 2.0;
  diag_max_freq_ = 20.0;
  diag_topic_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
    "velodyne_points", diagnostics_, diagnostic_updater::FrequencyStatusParam(
      &diag_min_freq_, &diag_max_freq_, 0.1, 10),
    diagnostic_updater::TimeStampStatusParam());

  data_->setParameters(min_range, max_range, view_direction, view_width);
  container_ptr_->configure(min_range, max_range, target_frame, fixed_frame);
}

/** @brief Callback for raw scan messages.
 *
 *  @pre TF message filter has already waited until the transform to
 *       the configured @c frame_id can succeed.
 */
void Transform::processScan(
  const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr scanMsg)
{
  if (output_->get_subscription_count() == 0 &&
    output_->get_intra_process_subscription_count() == 0)    // no one listening?
  {
    return;
  }

  container_ptr_->setup(scanMsg);

  // sufficient to calculate single transform for whole scan
  if (!container_ptr_->computeTransformToTarget(scanMsg->header.stamp)) {
    // target frame not available
    return;
  }

  // process each packet provided by the driver
  for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
    // calculate individual transform for each packet to account for ego
    // during one rotation of the velodyne sensor
    if (!container_ptr_->computeTransformToFixed(scanMsg->packets[i].stamp)) {
      // fixed frame not available
      return;
    }
    data_->unpack(scanMsg->packets[i], *container_ptr_, scanMsg->header.stamp);
  }

  // publish the accumulated cloud message
  output_->publish(container_ptr_->finishCloud());
  diag_topic_->tick(scanMsg->header.stamp);
}

}  // namespace velodyne_pointcloud

RCLCPP_COMPONENTS_REGISTER_NODE(velodyne_pointcloud::Transform)
