/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS nodelet converts new velodyne_msgs/VelodyneScan messages backwards
    to the old-format velodyne_common/RawScan.

*/

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <velodyne_common/RawScan.h>
#include <velodyne_msgs/VelodyneScan.h>

class ConvertBack: public nodelet::Nodelet
{
public:
  ConvertBack() {}
  ~ConvertBack() {}
  virtual void onInit();

private:
  void processScan(const velodyne_msgs::VelodyneScanPtr &scan);

  ros::Subscriber scan_;
  ros::Publisher output_;
};

void ConvertBack::onInit()
{
  ros::NodeHandle node = getNodeHandle();

  // subscribe to old-format raw scan data
  scan_ =
    node.subscribe("velodyne/packets", 1,
                   &ConvertBack::processScan, this,
                   ros::TransportHints().tcpNoDelay(true));

  output_ =
    node.advertise<velodyne_common::RawScan>("velodyne/rawscan", 10);
}

/** \brief callback for scan data
 *
 *  converts velodyne_msgs/VelodyneScan message to  velodyne_common/RawScan
 */
void ConvertBack::processScan(const velodyne_msgs::VelodyneScanPtr &scan)
{
  if (output_.getNumSubscribers() == 0)         // no one listening?
    return;

  // allocate a new shared pointer for zero-copy sharing with other nodelets
  velodyne_common::RawScanPtr msg(new velodyne_common::RawScan);

  // set time stamp and frame ID in output message header
  msg->header.stamp = scan->header.stamp ;
  msg->header.frame_id = scan->header.frame_id;

  size_t npackets = scan->packets.size();
  size_t packet_size = velodyne_common::RawScan::PACKET_SIZE;
  size_t nbytes = npackets * packet_size;

  // allocate all the packets for the output message
  msg->data.resize(nbytes);
  size_t next = 0;                      // next input byte
  for (size_t pkt = 0; pkt < npackets; ++pkt)
    {
      for (size_t i = 0; i < packet_size; ++i)
        {
          msg->data[next++] = scan->packets[pkt].data[i];
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
PLUGINLIB_DECLARE_CLASS(velodyne_common, ConvertBack,
                        ConvertBack, nodelet::Nodelet);
