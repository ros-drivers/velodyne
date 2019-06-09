/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node transforms raw Velodyne LIDAR packets to PointCloud2
    in the /map frame of reference.

*/

#include <rclcpp/rclcpp.hpp>
#include "velodyne_pointcloud/transform.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // create conversion class, which subscribes to raw data
  auto transform = std::make_shared<velodyne_pointcloud::Transform>();

  // handle callbacks until shut down
  rclcpp::spin(transform);
  rclcpp::shutdown();

  return 0;
}
