/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include "velodyne_pointcloud/convert.h"

#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include <velodyne_pointcloud/organized_cloudXYZIR.h>
#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh);


    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
      
    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));
  }
  
  void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
    config_.organize_cloud = config.organize_cloud;
    config_.min_range = config.min_range;
    config_.max_range = config.max_range;
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    velodyne_rawdata::DataContainerBase* container_ptr;
    if(config_.organize_cloud)
    {
      container_ptr = new OrganizedCloudXYZIR();
      container_ptr->pc->width = config_.num_lasers;
    }
    else
    {
      container_ptr = new PointcloudXYZIR();
      // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
      container_ptr->pc->height = 1;
      container_ptr->pc->is_dense = false;
    }

    container_ptr->setParameters(config_.min_range, config_.max_range, scanMsg->header.frame_id, scanMsg->header.frame_id);
    container_ptr->pc->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    container_ptr->pc->header.frame_id = scanMsg->header.frame_id;

    // allocate a point cloud with same time and frame ID as raw data
    container_ptr->pc->points.reserve(scanMsg->packets.size() * data_->scansPerPacket());

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      data_->unpack(scanMsg->packets[i], *container_ptr);
    }

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << container_ptr->pc->height * container_ptr->pc->width
                     << " Velodyne points, time: " << container_ptr->pc->header.stamp);
    output_.publish(container_ptr->pc);
    delete container_ptr;
  }

} // namespace velodyne_pointcloud
