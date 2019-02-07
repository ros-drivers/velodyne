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
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <sensor_msgs/PointCloud2.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h>

#include <dynamic_reconfigure/server.h>
#include <velodyne_pointcloud/TransformNodeConfig.h>

// include template implementations to transform a custom point cloud
#include <pcl_ros/impl/transforms.hpp>

// instantiate template for transforming a VPointCloud
template bool
  pcl_ros::transformPointCloud<velodyne_rawdata::VPoint>(const std::string &,
                                       const velodyne_rawdata::VPointCloud &,
                                       velodyne_rawdata::VPointCloud &,
                                       const tf::TransformListener &);

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
    ros::Publisher output_;
    boost::shared_ptr<tf::MessageFilter<velodyne_msgs::VelodyneScan> >tf_filter_ptr_;
    boost::shared_ptr<tf::TransformListener> listener_ptr_;

    /// configuration parameters
    typedef struct {
      std::string target_frame;      ///< target frame
      std::string fixed_frame;       ///< fixed frame
      bool organize_cloud;           ///< enable/disable organized cloud structure
      double max_range;              ///< maximum range to publish
      double min_range;              ///< minimum range to publish
      uint16_t num_lasers;           ///< number of lasers
    } Config;
    Config config_;

    boost::shared_ptr<velodyne_rawdata::DataContainerBase> container_ptr;

  };

} // namespace velodyne_pointcloud

#endif // _VELODYNE_POINTCLOUD_TRANSFORM_H_
