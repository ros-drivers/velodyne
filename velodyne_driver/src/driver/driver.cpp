// Copyright 2007, 2009-2012, 2019 Austin Robot Technology, Patrick Beeson, Jack O'Quin, AutonomouStuff  // NOLINT
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

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include "velodyne_driver/driver.hpp"

#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

namespace velodyne_driver
{

VelodyneDriver::VelodyneDriver(const rclcpp::NodeOptions & options)
: rclcpp::Node("velodyne_driver_node", options),
  diagnostics_(this, 0.2)
{
  std::string devip = this->declare_parameter("device_ip", std::string(""));
  bool gps_time = this->declare_parameter("gps_time", false);

  rcl_interfaces::msg::ParameterDescriptor offset_desc;
  offset_desc.name = "time_offset";
  offset_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  offset_desc.description = "Time offset";
  rcl_interfaces::msg::FloatingPointRange offset_range;
  offset_range.from_value = -1.0;
  offset_range.to_value = 1.0;
  offset_desc.floating_point_range.push_back(offset_range);
  config_.time_offset = this->declare_parameter("time_offset", 0.0, offset_desc);

  config_.enabled = this->declare_parameter("enabled", true);
  bool read_once = this->declare_parameter("read_once", false);
  bool read_fast = this->declare_parameter("read_fast", false);
  double repeat_delay = this->declare_parameter("repeat_delay", 0.0);
  config_.frame_id = this->declare_parameter("frame_id", std::string("velodyne"));
  config_.model = this->declare_parameter("model", std::string("64E"));
  config_.rpm = this->declare_parameter("rpm", 600.0);
  std::string dump_file = this->declare_parameter("pcap", std::string(""));
  double cut_angle = this->declare_parameter("cut_angle", -1.0);
  int udp_port = this->declare_parameter("port", static_cast<int>(DATA_PORT_NUMBER));
  config_.timestamp_first_packet = this->declare_parameter("timestamp_first_packet", false);

  future_ = exit_signal_.get_future();

  // get model name, validate string, determine packet rate
  double packet_rate;                   // packet frequency (Hz)
  std::string model_full_name;
  if (config_.model == "VLS128") {
    packet_rate = 6253.9;   // 3 firing cycles in a data packet.
                            // 3 x 53.3 μs = 0.1599 ms is the accumulation delay per packet.
                            // 1 packet/0.1599 ms = 6253.9 packets/second

    model_full_name = config_.model;
  } else if ((config_.model == "64E_S2") || (config_.model == "64E_S2.1")) {
    // generates 1333312 points per second
    packet_rate = 3472.17;              // 1 packet holds 384 points - 1333312 / 384
    model_full_name = std::string("HDL-") + config_.model;
  } else if (config_.model == "64E") {
    packet_rate = 2600.0;
    model_full_name = std::string("HDL-") + config_.model;
  } else if (config_.model == "64E_S3") {  // generates 2222220 points per second
    packet_rate = 5787.03;                 // 1 packet holds 384 points - 2222220 / 384
    model_full_name = std::string("HDL-") + config_.model;
  } else if (config_.model == "32E") {
    packet_rate = 1808.0;
    model_full_name = std::string("HDL-") + config_.model;
  } else if (config_.model == "32C") {
    packet_rate = 1507.0;
    model_full_name = std::string("VLP-") + config_.model;
  } else if (config_.model == "VLP16") {
    packet_rate = 754;             // 754 Packets/Second for Last or Strongest mode 1508 for dual
    model_full_name = "VLP-16";
  } else {
    throw std::runtime_error("Unknown Velodyne LIDAR model: " + config_.model);
  }

  std::string deviceName(std::string("Velodyne ") + model_full_name);

  RCLCPP_INFO(this->get_logger(), "%s rotating at %f RPM", deviceName.c_str(), config_.rpm);
  double frequency = (config_.rpm / 60.0);     // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = static_cast<int>(std::ceil(packet_rate / frequency));
  RCLCPP_INFO(this->get_logger(), "publishing %d packets per scan", config_.npackets);

  if (cut_angle < 0.0) {
    RCLCPP_INFO(this->get_logger(), "Cut at specific angle feature deactivated.");
  } else if (cut_angle <= (2.0 * M_PI)) {
    RCLCPP_INFO(
      this->get_logger(), "Cut at specific angle feature activated. "
      "Cutting velodyne points always at %f rad.", cut_angle);
  } else {
    RCLCPP_ERROR(
      this->get_logger(), "cut_angle parameter is out of range."
      "Allowed range is between 0.0 and 2*PI or negative values to deactivate this feature.");
    cut_angle = -0.01;
  }

  // if we are timestamping based on the first or last packet in the scan
  if (config_.timestamp_first_packet) {
    RCLCPP_INFO(
      this->get_logger(),
      "Setting velodyne scan start time to timestamp of first packet");
  }

  // Convert cut_angle from radian to one-hundredth degree,
  // which is used in velodyne packets
  config_.cut_angle = static_cast<int>((cut_angle * 360 / (2 * M_PI)) * 100);

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate / config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;

  RCLCPP_INFO(this->get_logger(), "expected frequency: %.3f (Hz)", diag_freq);

  diag_topic_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
    "velodyne_packets", diagnostics_, diagnostic_updater::FrequencyStatusParam(
      &diag_min_freq_, &diag_max_freq_, 0.1, 10),
    diagnostic_updater::TimeStampStatusParam());

  // open Velodyne input device or file
  if (!dump_file.empty()) {                // have PCAP file?
    // read data from packet capture file
    input_ = std::make_unique<velodyne_driver::InputPCAP>(
      this, devip, udp_port, packet_rate,
      dump_file, read_once, read_fast, repeat_delay);
  } else {
    // read data from live socket
    input_ = std::make_unique<velodyne_driver::InputSocket>(this, devip, udp_port, gps_time);
  }

  // raw packet output topic
  output_ =
    this->create_publisher<velodyne_msgs::msg::VelodyneScan>("velodyne_packets", 10);

  last_azimuth_ = -1;

  poll_thread_ = std::thread(&VelodyneDriver::pollThread, this);
}

VelodyneDriver::~VelodyneDriver()
{
  exit_signal_.set_value();
  poll_thread_.join();
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll()
{
  if (!config_.enabled) {
    // If we are not enabled exit once a second to let the caller handle
    // anything it might need to, such as if it needs to exit.
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return true;
  }

  // Allocate a new unique pointer for zero-copy sharing with other nodes.
  std::unique_ptr<velodyne_msgs::msg::VelodyneScan> scan =
    std::make_unique<velodyne_msgs::msg::VelodyneScan>();

  if (config_.cut_angle >= 0) {  // Cut at specific angle feature enabled
    scan->packets.reserve(config_.npackets);
    velodyne_msgs::msg::VelodynePacket tmp_packet;

    while (true) {
      while (true) {
        int rc = input_->getPacket(&tmp_packet, config_.time_offset);
        if (rc == 0) {  // got a full packet?
          break;
        }

        if (rc < 0) {  // end of file reached?
          return false;
        }
      }

      scan->packets.push_back(tmp_packet);

      // Extract base rotation of first block in packet
      size_t azimuth_data_pos = 100 * 0 + 2;
      int azimuth = *(reinterpret_cast<uint16_t *>(&tmp_packet.data[azimuth_data_pos]));

      // if first packet in scan, there is no "valid" last_azimuth_
      if (last_azimuth_ == -1) {
        last_azimuth_ = azimuth;
        continue;
      }

      if ((last_azimuth_ < config_.cut_angle && config_.cut_angle <= azimuth) ||
        (config_.cut_angle <= azimuth && azimuth < last_azimuth_) ||
        (azimuth < last_azimuth_ && last_azimuth_ < config_.cut_angle))
      {
        last_azimuth_ = azimuth;
        break;  // Cut angle passed, one full revolution collected
      }

      last_azimuth_ = azimuth;
    }
  } else {  // standard behaviour
    // Since the velodyne delivers data at a very high rate, keep
    // reading and publishing scans as fast as possible.
    scan->packets.resize(config_.npackets);

    for (int i = 0; i < config_.npackets; ++i) {
      while (true) {
        // keep reading until full packet received
        int rc = input_->getPacket(&scan->packets[i], config_.time_offset);
        if (rc == 0) {  // got a full packet?
          break;
        }

        if (rc < 0) {  // end of file reached?
          return false;
        }
      }
    }
  }

  // publish message using time of last packet read
  RCLCPP_DEBUG(this->get_logger(), "Publishing a full Velodyne scan.");
  builtin_interfaces::msg::Time stamp =
    config_.timestamp_first_packet ? scan->packets.front().stamp : scan->packets.back().stamp;
  scan->header.stamp = stamp;
  scan->header.frame_id = config_.frame_id;
  output_->publish(std::move(scan));

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(stamp);

  return true;
}

void VelodyneDriver::pollThread()
{
  std::future_status status;

  do {
    poll();
    status = future_.wait_for(std::chrono::seconds(0));
  } while (status == std::future_status::timeout);
}

}  // namespace velodyne_driver

RCLCPP_COMPONENTS_REGISTER_NODE(velodyne_driver::VelodyneDriver)
