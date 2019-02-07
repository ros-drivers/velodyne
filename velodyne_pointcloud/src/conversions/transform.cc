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

    @author Jack O'Quin
    @author Jesse Vera

*/

#include "velodyne_pointcloud/transform.h"

#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include <velodyne_pointcloud/organized_cloudXYZIR.h>
#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh):
    tf_prefix_(tf::getPrefixParam(private_nh)),
    data_(new velodyne_rawdata::RawData())
  {
    // Read calibration.
    data_->setup(private_nh);

    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      TransformNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::TransformNodeConfig>::
      CallbackType f;
    f = boost::bind (&Transform::reconfigure_callback, this, _1, _2);
    srv_->setCallback (f);
    
    // subscribe to VelodyneScan packets using transform filter
    velodyne_scan_.subscribe(node, "velodyne_packets", 10);
    tf_filter_ptr_ = boost::shared_ptr<tf::MessageFilter<velodyne_msgs::VelodyneScan> >(
        new tf::MessageFilter<velodyne_msgs::VelodyneScan>(velodyne_scan_, *listener_ptr_, config_.target_frame, 10));
    tf_filter_ptr_->registerCallback(boost::bind(&Transform::processScan, this, _1));
    private_nh.param<std::string>("fixed_frame", config_.fixed_frame, "odom");

    if(config_.organize_cloud)
    {
      container_ptr = boost::shared_ptr<OrganizedCloudXYZIR>(new OrganizedCloudXYZIR);
      container_ptr->pc->width = config_.num_lasers;
      container_ptr->pc->is_dense = false;
    }
    else
    {
      container_ptr = boost::shared_ptr<PointcloudXYZIR>(new PointcloudXYZIR);
      container_ptr->pc->height = 1;
      container_ptr->pc->is_dense = true;
    }
  }
  
  void Transform::reconfigure_callback(
      velodyne_pointcloud::TransformNodeConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM("Reconfigure request.");
    data_->setParameters(config.min_range, config.max_range, 
                         config.view_direction, config.view_width);
    config_.target_frame = tf::resolve(tf_prefix_, config.frame_id);
    ROS_INFO_STREAM("Target frame ID: " << config_.target_frame);
    config_.organize_cloud = config.organize_cloud;
    config_.min_range = config.min_range;
    config_.max_range = config.max_range;

    if(config_.organize_cloud)
    {
      container_ptr = boost::shared_ptr<OrganizedCloudXYZIR>(new OrganizedCloudXYZIR);
      container_ptr->pc->width = config_.num_lasers;
      container_ptr->pc->is_dense = false;
    }
    else
    {
      container_ptr = boost::shared_ptr<PointcloudXYZIR>(new PointcloudXYZIR);
      container_ptr->pc->height = 1;
      container_ptr->pc->is_dense = true;
    }
    container_ptr->setParameters(
        config_.min_range, config_.max_range,
        config_.target_frame, config_.fixed_frame, listener_ptr_);
  }

  /** @brief Callback for raw scan messages.
   *
   *  @pre TF message filter has already waited until the transform to
   *       the configured @c frame_id can succeed.
   */
  void
    Transform::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    container_ptr->reset();
    container_ptr->pc->header.stamp = pcl_conversions::toPCL(scanMsg->header.stamp);
    container_ptr->pc->header.frame_id = scanMsg->header.frame_id;

    // allocate a point cloud with same time and frame ID as raw data
    container_ptr->pc->points.reserve(scanMsg->packets.size() * data_->scansPerPacket());

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      data_->unpack(scanMsg->packets[i], *container_ptr);
    }
    // did transformation of point in the container if necessary
    container_ptr->pc->header.frame_id = config_.target_frame;

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << container_ptr->pc->height * container_ptr->pc->width
                                   << " Velodyne points, time: " << container_ptr->pc->header.stamp);
    output_.publish(container_ptr->pc);
  }

} // namespace velodyne_pointcloud
