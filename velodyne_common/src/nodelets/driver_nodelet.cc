/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver nodelet for the Velodyne HDL-64E 3D LIDAR
 */

#include <string>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <velodyne/input.h>
#include <velodyne_common/RawScan.h>


class DriverNodelet: public nodelet::Nodelet
{
public:
  DriverNodelet()
    : input_(0)
  {}
  ~DriverNodelet()
  {
    if (input_) delete input_;
  }

private:
  virtual void onInit();

  velodyne::Input *input_;
  ros::Publisher output_;
};

void DriverNodelet::onInit()
{
  // use private node handle to get parameters
  ros::NodeHandle private_nh = getPrivateNodeHandle();
  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  if (dump_file != "")
    {
      NODELET_INFO("reading data from file: %s", dump_file.c_str());
      input_ = new velodyne::InputPCAP(dump_file);
    }
  else
    {
      NODELET_INFO("reading data from socket");
      input_ = new velodyne::InputSocket();
    }

  if (input_->getParams() != 0)
    {
      NODELET_FATAL("Failed to read Velodyne packet input parameters.");
      return;
    }

  ros::NodeHandle node = getNodeHandle();

  // open Velodyne input device or file
  if(input_->vopen() != 0)
    {
      NODELET_FATAL("Cannot open Velodyne input.");
      return;
    }

  output_ = node.advertise<velodyne_common::RawScan>("velodyne/rawscan", 1);

  int npackets = velodyne_common::RawScan::PACKETS_PER_REVOLUTION;

  // Loop until shut down.
  while(ros::ok())
    {
      double time;

      // allocate a new shared pointer for zero-copy sharing with other nodelets
      velodyne_common::RawScanPtr scan(new velodyne_common::RawScan);
      scan->data.resize(npackets * velodyne_common::RawScan::PACKET_SIZE);

      // Since the velodyne delivers data at a very high rate, keep
      // reading and publishing scans as fast as possible.
      int packets_left = input_->getPackets(&scan->data[0], npackets, &time);

      if (packets_left != 0)            // incomplete scan received?
        {
          if (packets_left < 0)         // end of file reached?
            break;

          if (packets_left < npackets)  // partial scan read?
            NODELET_WARN("Incomplete Velodyne scan: %d packets missing",
                     packets_left);
        }
      else
        {
          NODELET_DEBUG("Publishing a full Velodyne scan.");
          scan->header.stamp = ros::Time(time);
          scan->header.frame_id = "/velodyne";
          output_.publish(scan);
        }
    }

  input_->vclose();
}

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_common, DriverNodelet,
                        DriverNodelet, nodelet::Nodelet);
