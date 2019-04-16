// Copyright (C) 2007, 2009-2012 Austin Robot Technology, Patrick Beeson, Jack O'Quin
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

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include "velodyne_driver/driver.h"

using namespace std::chrono_literals;

namespace velodyne_driver
{

VelodyneDriver::VelodyneDriver() : rclcpp::Node("velodyne_node")
{
  // use private node handle to get parameters
  this->get_parameter_or("frame_id", config_.frame_id, std::string("velodyne"));

  // get model name, validate string, determine packet rate
  this->get_parameter_or("model", config_.model, std::string("64E"));
  double packet_rate;                   // packet frequency (Hz)
  std::string model_full_name;
  if ((config_.model == "64E_S2") || 
      (config_.model == "64E_S2.1"))    // generates 1333312 points per second
    {                                   // 1 packet holds 384 points
      packet_rate = 3472.17;            // 1333312 / 384
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "64E")
    {
      packet_rate = 2600.0;
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "64E_S3") // generates 2222220 points per second (half for strongest and half for lastest)
    {                                 // 1 packet holds 384 points
      packet_rate = 5787.03;          // 2222220 / 384
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "32E")
    {
      packet_rate = 1808.0;
      model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "32C")
    {
      packet_rate = 1507.0;
      model_full_name = std::string("VLP-") + config_.model;
    }
  else if (config_.model == "VLP16")
    {
      packet_rate = 754;             // 754 Packets/Second for Last or Strongest mode 1508 for dual (VLP-16 User Manual)
      model_full_name = "VLP-16";
    }
  else
    {
      RCLCPP_ERROR(this->get_logger(), "unknown Velodyne LIDAR model: " + config_.model);
      packet_rate = 2600.0;
    }
  std::string deviceName(std::string("Velodyne ") + model_full_name);

  this->get_parameter_or("rpm", config_.rpm, 600.0);
  RCLCPP_INFO(this->get_logger(), "%s rotating at %d RPM", deviceName.c_str(), config_.rpm);
  double frequency = (config_.rpm / 60.0);     // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = (int) ceil(packet_rate / frequency);
  this->get_parameter("npackets", config_.npackets);
  RCLCPP_INFO(this->get_logger(), "publishing " + std::to_string(config_.npackets) + " packets per scan");

  std::string dump_file;
  this->get_parameter_or("pcap", dump_file, std::string(""));

  double cut_angle;
  this->get_parameter_or("cut_angle", cut_angle, -0.01);
  if (cut_angle < 0.0)
  {
    RCLCPP_INFO(this->get_logger(), "Cut at specific angle feature deactivated.");
  }
  else if (cut_angle < (2*M_PI))
  {
      RCLCPP_INFO(this->get_logger(), "Cut at specific angle feature activated. "
            "Cutting velodyne points always at " + std::to_string(cut_angle) + " rad.");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(),"cut_angle parameter is out of range."
                 "Allowed range is between 0.0 and 2*PI or negative values to deactivate this feature.");
    cut_angle = -0.01;
  }
  
  // Convert cut_angle from radian to one-hundredth degree,
  // which is used in velodyne packets
  config_.cut_angle = int((cut_angle*360/(2*M_PI))*100);

  int udp_port;
  this->get_parameter_or("port", udp_port, (int) DATA_PORT_NUMBER);
  
  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate/config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  RCLCPP_INFO(this->get_logger(), "expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("velodyne_packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             0.1, 10),
                                        TimeStampStatusParam()));
  diag_timer_  = this->create_wall_timer(200ms, std::bind(&VelodyneDriver::diagTimerCallback, this));
  // open Velodyne input device or file
  if (dump_file != "")                  // have PCAP file?
    {
      // read data from packet capture file
      input_.reset(new velodyne_driver::InputPCAP(shared_from_this() , udp_port,
                                                  packet_rate, dump_file));
    }
  else
    {
      
      // read data from live socket
      input_.reset(new velodyne_driver::InputSocket(shared_from_this() , udp_port));
    }
  
  // raw packet output topic
  output_ =
    this->create_publisher<velodyne_msgs::msg::VelodyneScan>("~/velodyne_packets", 10);

  last_azimuth_ = -1;
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll(void)
{
  // Allocate a new shared pointer for zero-copy sharing with other nodes.
  std::shared_ptr<velodyne_msgs::msg::VelodyneScan> scan = std::make_shared<velodyne_msgs::msg::VelodyneScan>();
  if( config_.cut_angle >= 0) //Cut at specific angle feature enabled
  {
    scan->packets.reserve(config_.npackets);
    velodyne_msgs::msg::VelodynePacket tmp_packet;
    while(true)
    {
      while(true)
      {
        int rc = input_->getPacket(&tmp_packet, config_.time_offset);
        if (rc == 0) break;       // got a full packet?
        if (rc < 0) return false; // end of file reached?
      }
      scan->packets.push_back(tmp_packet);

      // Extract base rotation of first block in packet
      std::size_t azimuth_data_pos = 100*0+2;
      int azimuth = *( (u_int16_t*) (&tmp_packet.data[azimuth_data_pos]));

      //if first packet in scan, there is no "valid" last_azimuth_
      if (last_azimuth_ == -1) {
      	 last_azimuth_ = azimuth;
      	 continue;
      }
      if((last_azimuth_ < config_.cut_angle && config_.cut_angle <= azimuth)
      	 || ( config_.cut_angle <= azimuth && azimuth < last_azimuth_)
      	 || (azimuth < last_azimuth_ && last_azimuth_ < config_.cut_angle))
      {
        last_azimuth_ = azimuth;
        break; // Cut angle passed, one full revolution collected
      }
      last_azimuth_ = azimuth;
    }
  }
  else // standard behaviour
  {
  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
    scan->packets.resize(config_.npackets);
    for (int i = 0; i < config_.npackets; ++i)
    {
      while (true)
        {
          // keep reading until full packet received
          int rc = input_->getPacket(&scan->packets[i], config_.time_offset);
          if (rc == 0) break;       // got a full packet?
          if (rc < 0) return false; // end of file reached?
        }
    }
  }

  // publish message using time of last packet read
  RCLCPP_DEBUG(this->get_logger(), "Publishing a full Velodyne scan.");
  scan->header.stamp = scan->packets.back().stamp;
  scan->header.frame_id = config_.frame_id;
  output_->publish(scan);

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.update();

  return true;
}

void VelodyneDriver::diagTimerCallback()
{
  // Call necessary to provide an error when no velodyne packets are received
  diagnostics_.update();
}

} // namespace velodyne_driver

//#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
//RCLCPP_COMPONENTS_REGISTER_NODE(VelodyneDriver::VelodyneDriver)
