/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This ROS node converts a Velodyne 3D LIDAR PointXYZIR cloud to
    PointXYZRGB, assigning colors for visualization of the laser
    rings.

*/

#include <rclcpp/rclcpp.hpp>
#include "velodyne_pointcloud/colors.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // create conversion class, which subscribes to raw data
  auto colors = std::make_shared<velodyne_pointcloud::RingColors>();
  
  // handle callbacks until shut down
  rclcpp::spin(colors);
  rclcpp::shutdown();
  
  return 0;
}
