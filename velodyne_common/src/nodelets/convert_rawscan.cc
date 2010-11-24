/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS nodelet converts old-format velodyne_common/RawScan
    messages to the new velodyne_msgs/VelodyneScan.

*/

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <velodyne_common/RawScan.h>
#include <velodyne_msgs/VelodyneScan.h>

class ConvertRawScan: public nodelet::Nodelet
{
public:
  ConvertRawScan() {}
  ~ConvertRawScan() {}
  virtual void onInit();

private:
  void processScan(const velodyne_common::RawScanPtr &scan);

  ros::Subscriber raw_scan_;
  ros::Publisher output_;
};

void ConvertRawScan::onInit()
{
  ros::NodeHandle node = getNodeHandle();

  // subscribe to old-format raw scan data
  raw_scan_ =
    node.subscribe("velodyne/rawscan", 1,
                   &ConvertRawScan::processScan, this,
                   ros::TransportHints().tcpNoDelay(true));

  output_ =
    node.advertise<velodyne_msgs::VelodyneScan>("velodyne/packets", 10);
}

/** \brief callback for scan data
 *
 *  converts velodyne_common/RawScan message to velodyne_msgs/VelodyneScan
 */
void ConvertRawScan::processScan(const velodyne_common::RawScanPtr &scan)
{
  if (output_.getNumSubscribers() == 0)         // no one listening?
    return;

  // allocate a new shared pointer for zero-copy sharing with other nodelets
  velodyne_msgs::VelodyneScanPtr msg(new velodyne_msgs::VelodyneScan);

  // set time stamp and frame ID in output message header
  msg->header.stamp = scan->header.stamp ;
  msg->header.frame_id = scan->header.frame_id;

  size_t nbytes = scan->data.size();
  size_t packet_size = velodyne_common::RawScan::PACKET_SIZE;
  size_t npackets = nbytes / packet_size;

  // allocate all the packets for the output message
  msg->packets.resize(npackets);
  size_t next = 0;                      // next input byte
  for (size_t pkt = 0; pkt < npackets; ++pkt)
    {
      msg->packets[pkt].stamp = scan->header.stamp;
      for (size_t i = 0; i < packet_size; ++i)
        {
          msg->packets[pkt].data[i] = scan->data[next++];
        }
    }

  // publish the reformatted message
  ROS_ASSERT(next == nbytes);           // got them all?
  NODELET_DEBUG_STREAM("Publishing " << nbytes << " Velodyne points.");
  output_.publish(msg);
}

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_common, ConvertRawScan,
                        ConvertRawScan, nodelet::Nodelet);
