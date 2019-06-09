/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node converts raw Velodyne LIDAR packets to PointCloud2.

*/

#include <rclcpp/rclcpp.hpp>
#include "velodyne_pointcloud/convert.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // create conversion class, which subscribes to raw data
  auto conv = std::make_shared<velodyne_pointcloud::Convert>();

  // handle callbacks until shut down
  rclcpp::spin(conv);
  
  rclcpp::shutdown();
  return 0;
}
