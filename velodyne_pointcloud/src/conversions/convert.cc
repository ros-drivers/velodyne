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

#include "convert.h"

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
      VelodyneConfigConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::VelodyneConfigConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));
  }
  
  void Convert::callback(velodyne_pointcloud::VelodyneConfigConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    velodyne_rawdata::VPointCloud::Ptr
      outMsg(new velodyne_rawdata::VPointCloud());
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg->header.frame_id = scanMsg->header.frame_id;
    outMsg->height = 1;

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
      {
        data_->unpack(scanMsg->packets[i], *outMsg);
      }
    
    // Rearrange the point cloud to render it organized.
    organizePointCloud(outMsg, data_->getNumLasers());

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg->height << " x " <<  outMsg->width
                     << " Velodyne points, time: " << outMsg->header.stamp);
    output_.publish(outMsg);
  }
  
  void Convert::organizePointCloud(
      velodyne_rawdata::VPointCloud::Ptr& pc, int numLasers)
  {
    // Rearrange only data coming from VLP-16.
    if (numLasers != velodyne_rawdata::VLP16_SCANS_PER_FIRING)
      return;
    
    // Allocate the organized point cloud.
    int height = velodyne_rawdata::VLP16_SCANS_PER_FIRING;
    int width  = pc->width*pc->height / height;
    if (pc->width*pc->height % height > 0) {
      width++;
    }
    
    velodyne_rawdata::VPoint defaultPointValue;
    defaultPointValue.x         = std::numeric_limits<float>::infinity();
    defaultPointValue.y         = std::numeric_limits<float>::infinity();
    defaultPointValue.z         = std::numeric_limits<float>::infinity();
    defaultPointValue.intensity = 0.0f;
    defaultPointValue.ring      = std::numeric_limits<uint16_t>::infinity();
    
    velodyne_rawdata::VPointCloud::Ptr organizedPc(
          new velodyne_rawdata::VPointCloud(width, height, defaultPointValue));
    organizedPc->header = pc->header;
    
    // Copy the given point cloud to the organized cloud.
    for (int p = 0; p < pc->size(); ++p) {
      
      // Point clouds are organized from upper left to lower right.
      // Compute the row and column of the point cloud.
      int col = p / velodyne_rawdata::VLP16_SCANS_PER_FIRING;
      int row = 15 - pc->at(p).ring;
      organizedPc->at(col, row) = pc->at(p);
    }
    
    pc = organizedPc;
  }

} // namespace velodyne_pointcloud
