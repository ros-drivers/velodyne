/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <velodyne_msgs/VelodyneScan.h>

#include "driver.h"

namespace velodyne_driver
{

VelodyneDriver::VelodyneDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
{
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("64E"));
  double packet_rate;                   // packet frequency (Hz)
  if ((config_.model == "64E_S2") || 
      (config_.model == "64E_S2.1"))    // generates 1333312 points per second
    {                                   // 1 packet holds 384 points
      packet_rate = 3472.17;            // 1333312 / 384
    }
  else if (config_.model == "64E")
    {
      packet_rate = 2600.0;
    }
  else if (config_.model == "32E")
    {
      packet_rate = 1808.0;
    }
  else
    {
      ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << config_.model);
      packet_rate = 2600.0;
    }
  std::string deviceName("Velodyne HDL-" + config_.model);

  private_nh.param("rpm", config_.rpm, 600.0);
  ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);     // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = (int) ceil(packet_rate / frequency);
  private_nh.getParam("npackets", config_.npackets);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  double cut_angle;
  private_nh.param("cut_angle", cut_angle, -0.01);
  config_.cut_angle = int(cut_angle*100);

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate/config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("velodyne_packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             0.1, 10),
                                        TimeStampStatusParam()));

  // open Velodyne input device or file
  if (dump_file != "")
    {
      input_.reset(new velodyne_driver::InputPCAP(private_nh,
                                                  packet_rate,
                                                  dump_file));
    }
  else
    {
      input_.reset(new velodyne_driver::InputSocket(private_nh));
    }

  // raw data output topic
  output_ = node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll(void)
{
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);

  if( config_.cut_angle >= 0) //Cut at specific angle feature enabled
  {
    scan->packets.reserve(config_.npackets);
    velodyne_msgs::VelodynePacket tmp_packet;
    while(true)
    {
      while(true)
      {
        int rc = input_->getPacket(&tmp_packet);
        if (rc == 0) break;       // got a full packet?
        if (rc < 0) return false; // end of file reached?
      }
      scan->packets.push_back(tmp_packet);

      static int last_base_rotation = -1;
      // Extract base rotation of first block in packet
      std::size_t rot_data_pos = 100*0+2;
      int base_rotation = *( (u_int16_t*) (&tmp_packet.data[rot_data_pos]));
      // Handle overflow 35999->0
      if(base_rotation<last_base_rotation)
        last_base_rotation-=36000;
      // Check if currently passing cut angle
      if(   last_base_rotation != -1
         && last_base_rotation < config_.cut_angle
         && base_rotation >= config_.cut_angle )
      {
        last_base_rotation = base_rotation;
        break; // Cut angle passed, one full revolution collected
      }
      last_base_rotation = base_rotation;
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
          int rc = input_->getPacket(&scan->packets[i]);
          if (rc == 0) break;       // got a full packet?
          if (rc < 0) return false; // end of file reached?
        }
    }
  }

  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full Velodyne scan.");
  scan->header.stamp = ros::Time(scan->packets[config_.npackets - 1].stamp);
  scan->header.frame_id = config_.frame_id;
  output_.publish(scan);

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.update();

  return true;
}

} // namespace velodyne_driver
