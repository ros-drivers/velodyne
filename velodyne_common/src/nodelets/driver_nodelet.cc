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
#include <tf/transform_listener.h>

#include <velodyne/input.h>
#include <velodyne_msgs/VelodyneScan.h>

class DriverNodelet: public nodelet::Nodelet
{
public:
  DriverNodelet()
  {
    input_ = NULL;
    running_ = false;
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
  std::string frame_id_;                ///< tf frame ID
  velodyne::Input *input_;
  ros::Publisher output_;
  boost::shared_ptr<boost::thread> deviceThread_;
};

void DriverNodelet::onInit()
{
  // use private node handle to get parameters
  ros::NodeHandle private_nh = getPrivateNodeHandle();

  private_nh.param("frame_id", frame_id_, std::string("velodyne"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  frame_id_ = tf::resolve(tf_prefix, frame_id_);

  private_nh.param("npackets", npackets_, (int)
                   velodyne_msgs::VelodyneScan::PACKETS_PER_REVOLUTION);

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
      // there should be an exception to throw here
      return;
    }

  output_ = node.advertise<velodyne_msgs::VelodyneScan>("velodyne/packets", 10);

  // spawn device thread
  running_ = true;
  deviceThread_ = boost::shared_ptr< boost::thread >
    (new boost::thread(boost::bind(&DriverNodelet::devicePoll, this)));
}

void DriverNodelet::devicePoll()
{
  // Loop until shut down or end of file
  while(running_)
    {
      // Allocate a new shared pointer for zero-copy sharing with other nodelets.
      velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);
      scan->packets.resize(npackets_);

      // Since the velodyne delivers data at a very high rate, keep
      // reading and publishing scans as fast as possible.
      for (int i = 0; i < npackets_; ++i)
        {
          while (true)
            {
              // keep reading until full packet received
              int rc = input_->getPacket(&scan->packets[i]);
              if (rc == 0) break;       // got a full packet?
              if (rc < 0) return;       // end of file reached?
            }
        }

      // publish message using time of last packet read
      NODELET_DEBUG("Publishing a full Velodyne scan.");
      scan->header.stamp = ros::Time(scan->packets[npackets_ - 1].stamp);
      scan->header.frame_id = frame_id_;
      output_.publish(scan);
    }
}

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_common, DriverNodelet,
                        DriverNodelet, nodelet::Nodelet);
