

#ifndef __POINTCLOUDXYZIR_H
#define __POINTCLOUDXYZIR_H

#include <velodyne_pointcloud/datacontainerbase.h>
#include <pcl_ros/point_cloud.h>
#include <velodyne_pointcloud/point_types.h>

namespace velodyne_pointcloud 
{
  // Shorthand typedefs for point cloud representations
  typedef velodyne_pointcloud::PointXYZIR VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  class PointcloudXYZIR : public velodyne_rawdata::DataContainerBase 
  {
  public:
    VPointCloud::Ptr pc;

    PointcloudXYZIR() : pc(new VPointCloud) {}
  
    virtual void addPoint(const float& x, const float& y, const float& z, const uint16_t& ring, const uint16_t& azimuth, const float& distance, const float& intensity);
  };
}
#endif //__POINTCLOUDXYZIR_H

