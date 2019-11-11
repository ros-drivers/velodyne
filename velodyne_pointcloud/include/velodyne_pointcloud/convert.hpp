// Copyright 2009, 2010, 2011, 2012, 2019 Austin Robot Technology, Jack O'Quin, Jesse Vera, Joshua Whitley  // NOLINT
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

#ifndef VELODYNE_POINTCLOUD__CONVERT_HPP_
#define VELODYNE_POINTCLOUD__CONVERT_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <memory>
#include <string>

#include "velodyne_pointcloud/pointcloudXYZIR.hpp"
#include "velodyne_pointcloud/rawdata.hpp"

namespace velodyne_pointcloud
{
class Convert final
  : public rclcpp::Node
{
public:
  explicit Convert(const rclcpp::NodeOptions & options);
  ~Convert() override {}
  Convert(Convert && c) = delete;
  Convert & operator=(Convert && c) = delete;
  Convert(const Convert & c) = delete;
  Convert & operator=(const Convert & c) = delete;

private:
  void processScan(const velodyne_msgs::msg::VelodyneScan::SharedPtr scanMsg);

  std::unique_ptr<velodyne_rawdata::RawData> data_;
  rclcpp::Subscription<velodyne_msgs::msg::VelodyneScan>::SharedPtr velodyne_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<velodyne_rawdata::DataContainerBase> container_ptr_;

  /// configuration parameters
  struct Config
  {
    int npackets;  // number of packets to combine
  };
  Config config_{};

  // diagnostics updater
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
};
}  // namespace velodyne_pointcloud

#endif  // VELODYNE_POINTCLOUD__CONVERT_HPP_
