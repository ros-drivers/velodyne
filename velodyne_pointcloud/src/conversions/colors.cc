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

#include "colors.h"
#include <velodyne_pointcloud/point_types.h>

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
  /** @brief Constructor. */
  RingColors::RingColors(ros::NodeHandle node, ros::NodeHandle private_nh)
  {
    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_rings", 10);

    // subscribe to VelodyneScan packets
    input_ =
      node.subscribe("velodyne_points", 10,
                     &RingColors::convertPoints, this,
                     ros::TransportHints().tcpNoDelay(true));
  }


  /** @brief Callback for Velodyne PointXYZRI messages. */
  void
  RingColors::convertPoints(const sensor_msgs::PointCloud2ConstPtr &inMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // do nothing

    // allocate an PointXYZRGB message with same time and frame ID as
    // input data
    sensor_msgs::PointCloud2::Ptr outMsg(new sensor_msgs::PointCloud2());
    sensor_msgs::PointCloud2Modifier modifier(*outMsg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(inMsg->height * inMsg->width);

    outMsg->header.stamp = inMsg->header.stamp;
    outMsg->header.frame_id = inMsg->header.frame_id;
    outMsg->height = 1;

    sensor_msgs::PointCloud2Iterator<float> out_x(*outMsg, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(*outMsg, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(*outMsg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> out_r(*outMsg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> out_g(*outMsg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> out_b(*outMsg, "b");

    sensor_msgs::PointCloud2ConstIterator<float> in_x(*inMsg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(*inMsg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(*inMsg, "z");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> in_ring(*inMsg, "ring");

    for (size_t i = 0; i < inMsg->height * inMsg->width; ++i, ++out_x, ++out_y, ++out_z, ++out_r, ++out_g, ++out_b,
      ++in_x, ++in_y, ++in_z, ++in_ring)
      {
        *out_x = *in_x;
        *out_y = *in_y;
        *out_z = *in_z;

        // color lasers with the rainbow array
        int color = *in_ring % N_COLORS;
        *out_r = (rainbow[color] >> 16) & 0x0000ff;
        *out_g = (rainbow[color] >> 8) & 0x0000ff;
        *out_b = rainbow[color] & 0x0000ff;
      }

    output_.publish(outMsg);
  }

} // namespace velodyne_pcl
