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
    data_(new velodyne_rawdata::RawData()), first_rcfg_call(true)
  {

    boost::optional<velodyne_pointcloud::Calibration> calibration = data_->setup(private_nh);
    if(calibration)
    {
        ROS_DEBUG_STREAM("Calibration file loaded.");
        config_.num_lasers = static_cast<uint16_t>(calibration.get().num_lasers);
    }
    else
    {
        ROS_ERROR_STREAM("Could not load calibration file!");
    }

    config_.target_frame = config_.fixed_frame = "velodyne";

    if(config_.organize_cloud)
    {
      container_ptr_ = boost::shared_ptr<OrganizedCloudXYZIR>(
          new OrganizedCloudXYZIR(config_.max_range, config_.min_range,
              config_.target_frame, config_.fixed_frame,
              config_.num_lasers, data_->scansPerPacket()));
    }
    else
    {
      container_ptr_ = boost::shared_ptr<PointcloudXYZIR>(
          new PointcloudXYZIR(config_.max_range, config_.min_range,
              config_.target_frame, config_.fixed_frame,
              data_->scansPerPacket()));
    }


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
    config_.min_range = config.min_range;
    config_.max_range = config.max_range;

    if(first_rcfg_call || config.organize_cloud != config_.organize_cloud){
        boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);
        config_.organize_cloud = config.organize_cloud;
        if(config_.organize_cloud) // TODO only on change
        {
            ROS_INFO_STREAM("Using the organized cloud format...");
            container_ptr_ = boost::shared_ptr<OrganizedCloudXYZIR>(
                new OrganizedCloudXYZIR(config_.max_range, config_.min_range,
                    config_.target_frame, config_.fixed_frame,
                    config_.num_lasers, data_->scansPerPacket()));
        }
        else
        {
            container_ptr_ = boost::shared_ptr<PointcloudXYZIR>(
                new PointcloudXYZIR(config_.max_range, config_.min_range,
                    config_.target_frame, config_.fixed_frame,
                    data_->scansPerPacket()));
        }
    }
    first_rcfg_call = false;

    container_ptr_->configure(config_.max_range, config_.min_range, config_.fixed_frame, config_.target_frame);

  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);
    // allocate a point cloud with same time and frame ID as raw data
    container_ptr_->setup(scanMsg);

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      data_->unpack(scanMsg->packets[i], *container_ptr_);
    }

    // did transformation of point in the container if necessary
    container_ptr_->pc->header.frame_id = config_.target_frame;

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << container_ptr_->pc->height * container_ptr_->pc->width
                     << " Velodyne points, time: " << container_ptr_->pc->header.stamp);
    output_.publish(container_ptr_->pc);
  }

} // namespace velodyne_pointcloud
