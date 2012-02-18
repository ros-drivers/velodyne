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
#include <pcl/io/pcd_io.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawDataXYZ())
  {
    private_nh.param("max_range", config_.max_range,
                     (double) velodyne_rawdata::DISTANCE_MAX);
    private_nh.param("min_range", config_.min_range, 2.0);
    ROS_INFO_STREAM("data ranges to publish: ["
                    << config_.min_range << ", "
                    << config_.max_range << "]");
    min_range2_ = config_.min_range * config_.min_range;
    max_range2_ = config_.max_range * config_.max_range;

    private_nh.param("npackets", config_.npackets,
                     velodyne_rawdata::PACKETS_PER_REV);
    ROS_INFO_STREAM("number of packets to accumulate: " << config_.npackets);

    data_->setup(private_nh);

    // allocate space for the output point cloud data
    pc_.points.reserve(config_.npackets*velodyne_rawdata::SCANS_PER_PACKET);
    pc_.points.clear();
    pc_.width = 0;
    pc_.height = 1;
    pc_.is_dense = true;
    packetCount_ = 0;

    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne/pointcloud2", 10);

#ifdef DATA_SUBSCRIBE
    // subscribe via RawData method
    velodyne_scan_ =
      data_->subscribe(node, "velodyne/packets", 10,
                       boost::bind(&Convert::processXYZ,
                                   this, _1, _2, _3),
                       ros::TransportHints().tcpNoDelay(true));
#else // DATA_SUBSCRIBE
    // subscribe directly to VelodyneScan
    velodyne_scan_ =
      node.subscribe("velodyne/packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));
#endif // DATA_SUBSCRIBE
  }

#ifndef DATA_SUBSCRIBE

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    // publish an empty point cloud message (test scaffolding)
    {
      sensor_msgs::PointCloud2Ptr outMsg(new sensor_msgs::PointCloud2());
      //pcl::toROSMsg(pc_, *outMsg);

      // publish the point cloud with same time and frame ID as raw data
      outMsg->header.stamp = scanMsg->header.stamp;
      outMsg->header.frame_id = scanMsg->header.frame_id;

      ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                       << " Velodyne points, time: "
                       << outMsg->header.stamp);
      output_.publish(outMsg);

      //pc_.points.clear();
      //pc_.width = 0;
      //packetCount_ = 0;
    }
  }

#else // DATA_SUBSCRIBE

  /** \brief Callback for XYZ points.
   *
   *  Converts Velodyne data for a single packet into a point cloud.
   *  Collects packets into a larger message (generally a full
   *  revolution).  Periodically publishes collected data as a
   *  PointCloud2 message.
   *
   *  @deprecated
   */
  void Convert::processXYZ(const velodyne_rawdata::xyz_scans_t &scan,
                                 ros::Time stamp,
                                 const std::string &frame_id)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work
    
    // fill in point values
    size_t npoints = scan.size();
    for (size_t i = 0; i < npoints; ++i)
    {
      velodyne_pointcloud::PointXYZIR p;
      p.x = scan[i].x;
      p.y = scan[i].y;
      p.z = scan[i].z;
      if (pointInRange(p))
        {
          p.intensity = scan[i].intensity;
          p.ring = velodyne::LASER_RING[scan[i].laser_number];
          pc_.points.push_back(p);
          ++pc_.width;
        }
    }

    if (++packetCount_ >= config_.npackets)
      {
        // buffer is full, publish it
        sensor_msgs::PointCloud2Ptr outMsg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(pc_, *outMsg);

        // publish the accumulated point cloud
        outMsg->header.stamp = stamp;               // time of last packet
        outMsg->header.frame_id = frame_id;         // input frame ID

        ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                         << " Velodyne points.");
        output_.publish(outMsg);
        pc_.points.clear();
        pc_.width = 0;
        packetCount_ = 0;
      }
  }
#endif // DATA_SUBSCRIBE

} // namespace velodyne_pointcloud
