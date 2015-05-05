/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011, 2012 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: data_base.h 1554 2011-06-14 22:11:17Z jack.oquin $
 */

/** \file
 *
 *  Point Cloud Library point structures for Velodyne data.
 *
 *  @author Jesse Vera
 *  @author Jack O'Quin
 *  @author Piyush Khandelwal
 */

#ifndef __VELODYNE_POINTCLOUD_POINT_TYPES_H
#define __VELODYNE_POINTCLOUD_POINT_TYPES_H

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace velodyne_pointcloud
{
  /** Euclidean Velodyne coordinate, including intensity and ring number. */
/*  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;
*/

  inline
  void setVelodyneCloudFields(sensor_msgs::PointCloud2 &pc) {
    sensor_msgs::PointCloud2Modifier modifier(pc);
    modifier.setPointCloud2Fields(5, "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1, sensor_msgs::PointField::FLOAT32,
                                  "ring", 1, sensor_msgs::PointField::UINT16);
  }

  inline
  sensor_msgs::PointCloud2::Ptr createVelodyneCloud() {
    sensor_msgs::PointCloud2::Ptr pc(new sensor_msgs::PointCloud2());
    setVelodyneCloudFields(*pc);

    return pc;
  };
}; // namespace velodyne_pointcloud

#endif // __VELODYNE_POINTCLOUD_POINT_TYPES_H

