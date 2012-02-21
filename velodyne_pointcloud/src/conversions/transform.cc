/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class transforms raw Velodyne 3D LIDAR packets to PointCloud2
    in the /odom frame of reference.

*/

#include "transform.h"

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    private_nh.param("frame_id", config_.frame_id, std::string("odom"));
    std::string tf_prefix = tf::getPrefixParam(private_nh);
    config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);
    ROS_INFO_STREAM("target frame ID: " << config_.frame_id);

    data_->setup(private_nh);

    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne/pointcloud2", 10);

    // subscribe to VelodyneScan packets
    ///  @todo use tf filter subscription instead
    velodyne_scan_ =
      node.subscribe("velodyne/packets", 10,
                     &Transform::processScan, (Transform *) this,
                     ros::TransportHints().tcpNoDelay(true));
  }

  /** @brief Callback for raw scan messages. */
  void
    Transform::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    // allocate an output point cloud with same time as raw data
    VPointCloud::Ptr outMsg(new VPointCloud());
    outMsg->header.stamp = scanMsg->header.stamp;
    outMsg->header.frame_id = config_.frame_id;
    outMsg->height = 1;

    // clear input point cloud to handle this packet
    inPc_.points.clear();
    inPc_.width = 0;
    inPc_.height = 1;
    inPc_.header.stamp = scanMsg->header.stamp;
    inPc_.header.frame_id = scanMsg->header.frame_id;

    // process each packet provided by the driver
    for (size_t next = 0; next < scanMsg->packets.size(); ++next)
      {
        // unpack the raw data
        data_->unpack(scanMsg->packets[next], inPc_);

        // transform the packet point cloud into the target frame
        try
          {
            tfPc_.points.clear();           // is this needed?
            tfPc_.width = 0;
            ROS_DEBUG_STREAM("transforming from" << inPc_.header.frame_id
                             << " to " << config_.frame_id);
#if 0 // waiting for transform causes delay problems
            /// @todo wait for transform to be available
            listener_.waitForTransform(config_.frame_id, frame_id, stamp,
                                       ros::Duration(0.2));
            pcl_ros::transformPointCloud(config_.frame_id, inPc_, tfPc_,
                                         listener_);
#else // use the latest transform available, should usually work fine
            pcl_ros::transformPointCloud(inPc_.header.frame_id,
                                         ros::Time(0), inPc_,
                                         config_.frame_id,
                                         tfPc_, listener_);
#endif
          }
        catch (tf::TransformException ex)
          {
            // only log tf error once every 100 times
            ROS_WARN_THROTTLE(100, "%s", ex.what());
            continue;                   // skip this packet
          }

        // append transformed packet data to end of output message
        outMsg->points.insert(outMsg->points.end(),
                             tfPc_.points.begin(),
                             tfPc_.points.end());
        outMsg->width += tfPc_.points.size();
      }

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                     << " Velodyne points, time: " << outMsg->header.stamp);
    output_.publish(outMsg);
  }

} // namespace velodyne_pointcloud
