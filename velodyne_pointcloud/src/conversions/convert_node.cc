/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node converts raw Velodyne LIDAR packets to PointCloud2.

*/

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "velodyne_pointcloud/convert.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  // handle callbacks until shut down
  rclcpp::spin(std::make_shared<velodyne_pointcloud::Convert>(rclcpp::NodeOptions()));

  rclcpp::shutdown();

  return 0;
}
