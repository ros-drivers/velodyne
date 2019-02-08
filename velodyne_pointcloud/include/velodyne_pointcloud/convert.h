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

#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/rawdata.h>

#include <dynamic_reconfigure/server.h>
#include <velodyne_pointcloud/CloudNodeConfig.h>

namespace velodyne_pointcloud
{
  class Convert
  {
  public:

    Convert(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~Convert() {}

  private:
    
    void callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level);
    void processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);

    ///Pointer to dynamic reconfigure service srv_
    boost::shared_ptr<dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > srv_;
    
    boost::shared_ptr<velodyne_rawdata::RawData> data_;
    ros::Subscriber velodyne_scan_;
    ros::Publisher output_;

    boost::shared_ptr<velodyne_rawdata::DataContainerBase> container_ptr_;

    boost::mutex reconfigure_mtx_;

    /// configuration parameters
    typedef struct {
      std::string target_frame;      ///< target frame
      std::string fixed_frame;       ///< fixed frame
      bool organize_cloud;           ///< enable/disable organized cloud structure
      double max_range;              ///< maximum range to publish
      double min_range;              ///< minimum range to publish
      uint16_t num_lasers;           ///< number of lasers
      int npackets;                  ///< number of packets to combine
    } Config;
    Config config_;
    bool first_rcfg_call;
  };

} // namespace velodyne_pointcloud

#endif // _VELODYNE_POINTCLOUD_CONVERT_H_
