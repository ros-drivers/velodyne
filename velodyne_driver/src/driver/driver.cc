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

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <velodyne_msgs/VelodyneScan.h>

#include "driver.h"

void VelodyneDriver::shutdown(void)
{
  if (input_)
    {
      input_->vclose();
    }
}

void VelodyneDriver::startup(ros::NodeHandle node,
                             ros::NodeHandle private_nh)
{
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  private_nh.param("npackets", config_.npackets, (int)
                   velodyne_msgs::VelodyneScan::PACKETS_PER_REVOLUTION);
  private_nh.param("rpm", config_.rpm, 600.0);
  ROS_INFO_STREAM("Velodyne rotating at " << config_.rpm << " RPM");

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  if (dump_file != "")
    {
      ROS_INFO("reading data from file: %s", dump_file.c_str());
      input_.reset(new velodyne_driver::InputPCAP(dump_file));
    }
  else
    {
      ROS_INFO("reading data from socket");
      input_.reset(new velodyne_driver::InputSocket());
    }

  // initialize diagnostics
  diagnostics_.setHardwareID("Velodyne HDL");  // TODO: get serial number
  double frequency = (config_.rpm / 60.0);     // nominal Hz rate
  double epsilon = frequency * 0.1;            // 10% error margin
  diag_max_freq_ = frequency + epsilon;
  diag_min_freq_ = frequency - epsilon;
  ROS_INFO("frequency range: [%.3f, %.3f] Hz", diag_min_freq_, diag_max_freq_);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("velodyne_packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             0.1, 10),
                                        TimeStampStatusParam()));

  // open Velodyne input device or file
  if(input_->vopen() != 0)
    {
      ROS_FATAL("Cannot open Velodyne input.");
      // there should be an exception to throw here
      return;
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
  scan->packets.resize(config_.npackets);

  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
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
