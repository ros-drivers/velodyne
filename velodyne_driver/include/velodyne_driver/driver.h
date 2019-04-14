// Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
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

#ifndef VELODYNE_DRIVER_DRIVER_H
#define VELODYNE_DRIVER_DRIVER_H

#include <string>
#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/error_handling.h>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <velodyne_driver/input.h>

namespace velodyne_driver
{

class VelodyneDriver : public rclcpp::Node
{
public :
  VelodyneDriver();
  ~VelodyneDriver() {}

  bool poll(void);

private:
  // Callback for diagnostics update for lost communication with vlp
  void diagTimerCallback();

  // configuration parameters
  struct
  {
    std::string frame_id;            // tf frame ID
    std::string model;               // device model name
    int    npackets;                 // number of packets to collect
    double rpm;                      // device rotation rate (RPMs)
    int cut_angle;                   // cutting angle in 1/100°
    double time_offset;              // time in seconds added to each velodyne time stamp
  }
  config_;

  std::shared_ptr<Input> input_;
  /* use std::shared_ptr or ::SharedPtr? */
  rclcpp::Publisher<velodyne_msgs::msg::VelodyneScan>::SharedPtr output_;
  int last_azimuth_;

  /* diagnostics updater */
  rclcpp::TimerBase::SharedPtr diag_timer_;
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  std::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
};

}  // namespace velodyne_driver

#endif  // VELODYNE_DRIVER_DRIVER_H
