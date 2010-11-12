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
#include <boost/thread.hpp>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <velodyne/input.h>
#include <velodyne_common/RawScan.h>


class DriverNodelet: public nodelet::Nodelet
{
public:
  DriverNodelet()
  {
    input_ = NULL;
    running_ = false;
    npackets_ = velodyne_common::RawScan::PACKETS_PER_REVOLUTION;
  }

  ~DriverNodelet()
  {
    if (running_)
      {
        NODELET_INFO("shutting down driver thread");
        running_ = false;
        deviceThread_->join();
        NODELET_INFO("driver thread stopped");
      }
    if (input_)
      {
        input_->vclose();
        delete input_;
      }
  }

private:
  virtual void onInit();
  virtual void devicePoll();

  // configuration parameters
  int npackets_;

  bool running_;                        ///< device is running
  velodyne::Input *input_;
  ros::Publisher output_;
  boost::shared_ptr<boost::thread> deviceThread_;
};

void DriverNodelet::onInit()
{
  // use private node handle to get parameters
  ros::NodeHandle private_nh = getPrivateNodeHandle();

  private_nh.getParam("npackets", npackets_);

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

  ros::NodeHandle node = getNodeHandle();

  // open Velodyne input device or file
  if(input_->vopen() != 0)
    {
      NODELET_FATAL("Cannot open Velodyne input.");
      return;
    }

  output_ = node.advertise<velodyne_common::RawScan>("velodyne/rawscan", 1);

  // spawn device thread
  running_ = true;
  deviceThread_ = boost::shared_ptr< boost::thread >
    (new boost::thread(boost::bind(&DriverNodelet::devicePoll, this)));
}

void DriverNodelet::devicePoll()
{
  // Loop until shut down...
  while(running_)
    {
      double time;

      // Allocate a new shared pointer for zero-copy sharing with other nodelets.
      velodyne_common::RawScanPtr scan(new velodyne_common::RawScan);
      scan->data.resize(npackets_ * velodyne_common::RawScan::PACKET_SIZE);

      // Since the velodyne delivers data at a very high rate, keep
      // reading and publishing scans as fast as possible.
      int packets_left = input_->getPackets(&scan->data[0], npackets_, &time);

      if (packets_left != 0)            // incomplete scan received?
        {
          if (packets_left < 0)         // end of file reached?
            break;

          if (packets_left < npackets_)  // partial scan read?
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
}

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_common, DriverNodelet,
                        DriverNodelet, nodelet::Nodelet);
