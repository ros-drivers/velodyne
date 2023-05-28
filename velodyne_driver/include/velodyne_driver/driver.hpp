// Copyright 2012, 2019 Austin Robot Technology, Jack O'Quin, AutonomouStuff
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

#ifndef VELODYNE_DRIVER__DRIVER_HPP_
#define VELODYNE_DRIVER__DRIVER_HPP_

#include <future>
#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "velodyne_driver/input.hpp"

namespace velodyne_driver
{

class VelodyneDriver final : public rclcpp::Node
{
public:
  explicit VelodyneDriver(const rclcpp::NodeOptions & options);
  ~VelodyneDriver() override;
  VelodyneDriver(VelodyneDriver && c) = delete;
  VelodyneDriver & operator=(VelodyneDriver && c) = delete;
  VelodyneDriver(const VelodyneDriver & c) = delete;
  VelodyneDriver & operator=(const VelodyneDriver & c) = delete;

private:
  bool poll();

  void pollThread();

  // configuration parameters
  struct
  {
    std::string frame_id;            // tf frame ID
    std::string model;               // device model name
    int npackets;                    // number of packets to collect
    double rpm;                      // device rotation rate (RPMs)
    int cut_angle;                   // cutting angle in radians
    double time_offset;              // time in seconds added to each velodyne time stamp
    bool enabled;                    // polling is enabled
    bool timestamp_first_packet;     // timestamp based on first packet instead of last one
  }
  config_;

  std::unique_ptr<Input> input_;
  rclcpp::Publisher<velodyne_msgs::msg::VelodyneScan>::SharedPtr output_;
  int last_azimuth_;

  /* diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;

  // We use this future/promise pair to notify threads that we are shutting down
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;

  // The thread that deals with data
  std::thread poll_thread_;
};

}  // namespace velodyne_driver

#endif  // VELODYNE_DRIVER__DRIVER_HPP_
