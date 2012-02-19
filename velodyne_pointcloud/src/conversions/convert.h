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

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#ifndef _VELODYNE_POINTCLOUD_CONVERT_H_
#define _VELODYNE_POINTCLOUD_CONVERT_H_ 1

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/ring_sequence.h>
#include <velodyne_pointcloud/point_types.h>

namespace velodyne_pointcloud
{
  class Convert
  {
  public:

    Convert(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~Convert() {}

  private:

#ifdef DEPRECATED_RAWDATA         // use DEPRECATED methods & types
    void processXYZ(const velodyne_rawdata::xyz_scans_t &scan,
                    ros::Time stamp,
                    const std::string &frame_id);
#else  // use new methods
    void processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);
    void processPacket(const velodyne_msgs::VelodynePacket *pkt,
                       const std::string &frame_id);
#endif // DEPRECATED_RAWDATA     // define DEPRECATED methods & types

    /** in-line test whether a point is in range */
    bool pointInRange(const velodyne_pointcloud::PointXYZIR &point)
    {
      float sum2 = (point.x * point.x 
                    + point.y * point.y
                    + point.z * point.z);
      return (sum2 >= min_range2_ && sum2 <= max_range2_);
    }

    // store squares of minimum and maximum ranges for efficient comparison
    float max_range2_;
    float min_range2_;

    boost::shared_ptr<velodyne_rawdata::RawDataXYZ> data_;
    ros::Subscriber velodyne_scan_;
    ros::Publisher output_;

    /// configuration parameters
    typedef struct {
      double max_range;                ///< maximum range to publish
      double min_range;                ///< minimum range to publish
      int npackets;                    ///< number of packets to combine
    } Config;
    Config config_;

    // point cloud buffers for collecting points over time
    // (class members to avoid allocation and deallocation overhead)
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> pc_;
    int packetCount_;           ///< count of output packets collected
  };

} // namespace velodyne_pointcloud

#endif // _VELODYNE_POINTCLOUD_CONVERT_H_
