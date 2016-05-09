/* -*- mode: C++ -*- */
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

#ifndef _VELODYNE_POINTCLOUD_TRANSFORM_H_
#define _VELODYNE_POINTCLOUD_TRANSFORM_H_ 1

#include <ros/ros.h>
#include "message_filters/subscriber.h"
#include <sensor_msgs/PointCloud2.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>

#include <dynamic_reconfigure/server.h>
#include <velodyne_pointcloud/TransformNodeConfig.h>

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

namespace velodyne_pointcloud
{
  class Transform
  {
  public:

    Transform(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~Transform() {}

  private:

    void processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);

    ///Pointer to dynamic reconfigure service srv_
    boost::shared_ptr<dynamic_reconfigure::Server<velodyne_pointcloud::
      TransformNodeConfig> > srv_;
    void reconfigure_callback(velodyne_pointcloud::TransformNodeConfig &config,
                  uint32_t level);
    
    const std::string tf_prefix_;
    boost::shared_ptr<velodyne_rawdata::RawData> data_;
    message_filters::Subscriber<velodyne_msgs::VelodyneScan> velodyne_scan_;
    tf2_ros::MessageFilter<velodyne_msgs::VelodyneScan> *tf_filter_;
    ros::Publisher output_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;

    /// configuration parameters
    typedef struct {
      std::string frame_id;          ///< target frame ID
    } Config;
    Config config_;

    // Point cloud buffers for collecting points within a packet.  The
    // inPc_ and tfPc_ are class members only to avoid reallocation on
    // every message.
    sensor_msgs::PointCloud2 inPc_;              ///< input packet point cloud
    sensor_msgs::PointCloud2 tfPc_;              ///< transformed packet point cloud
  };

} // namespace velodyne_pointcloud

#endif // _VELODYNE_POINTCLOUD_TRANSFORM_H_
