/*
 *  Copyright (C) 2012 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    Implementation for converting a Velodyne 3D LIDAR PointXYZIR cloud
    to PointXYZRGB, assigning colors for visualization of the laser
    rings.

    @author Jack O'Quin
*/

#include "velodyne_pointcloud/colors.h"
#include <velodyne_pointcloud/point_types.h>

/// @todo make sure these includes are really necessary
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

namespace
{
  // RGB color values
  const int color_red =       0xff0000;
  const int color_orange =    0xff8800;
  const int color_yellow =    0xffff00;
  const int color_green =     0x00ff00;
  const int color_blue =      0x0000ff;
  const int color_violet =    0xff00ff;

  const int N_COLORS = 6;
  int rainbow[N_COLORS] = {color_red, color_orange, color_yellow,
                           color_green, color_blue, color_violet};
}

namespace velodyne_pointcloud
{

  /** types of output point and cloud */
  typedef pcl::PointXYZRGB RGBPoint;
  typedef pcl::PointCloud<RGBPoint> RGBPointCloud;

  /** @brief Constructor. */
  RingColors::RingColors() : rclcpp::Node()
  {
    // advertise output point cloud (before subscribing to input data)
    output_ =
      self->advertise<sensor_msgs::PointCloud2>("velodyne_rings", 10);

    // subscribe to VelodyneScan packets
    input_ =
      self->subscribe("velodyne_points", 10,
                     &RingColors::convertPoints, this,
                     ros::TransportHints().tcpNoDelay(true));
  }


  /** @brief Callback for Velodyne PointXYZRI messages. */
  void
    RingColors::convertPoints(const VPointCloud::ConstPtr &inMsg)
  {
    if (output_->getNumSubscribers() == 0)         // no one listening?
      return;                                     // do nothing

    // allocate an PointXYZRGB message with same time and frame ID as
    // input data
    RGBPointCloud::Ptr outMsg(new RGBPointCloud());
    outMsg->header.stamp = inMsg->header.stamp;
    outMsg->header.frame_id = inMsg->header.frame_id;
    outMsg->height = 1;

    for (size_t i = 0; i < inMsg->points.size(); ++i)
      {
        RGBPoint p;
        p.x = inMsg->points[i].x;
        p.y = inMsg->points[i].y;
        p.z = inMsg->points[i].z;

        // color lasers with the rainbow array
        int color = inMsg->points[i].ring % N_COLORS;
        p.rgb = *reinterpret_cast<float*>(rainbow+color);

        outMsg->points.push_back(p);
        ++outMsg->width;
      }

    output_->publish(outMsg);
  }

} // namespace velodyne_pcl

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(velodyne_pointcloud::RingColors)
