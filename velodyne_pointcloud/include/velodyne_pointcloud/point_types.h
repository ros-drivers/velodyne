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

#ifndef VELODYNE_POINTCLOUD_POINT_TYPES_H
#define VELODYNE_POINTCLOUD_POINT_TYPES_H

#include <pcl/point_types.h>

namespace velodyne_pointcloud
{
/** Euclidean Velodyne coordinate, including intensity and ring number. */
struct PointXYZIR
{
  float x;                         ///< x coord
  float y;                         ///< y coord
  float z;                         ///< z coord
  float intensity;                 ///< laser intensity reading
  uint16_t ring;                   ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
}
EIGEN_ALIGN16;

};  // namespace velodyne_pointcloud


POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))

#endif  // VELODYNE_POINTCLOUD_POINT_TYPES_H
