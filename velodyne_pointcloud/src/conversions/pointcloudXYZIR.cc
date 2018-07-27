


#include <velodyne_pointcloud/pointcloudXYZIR.h>

namespace velodyne_pointcloud 
{
  void PointcloudXYZIR::addPoint(const float& x, const float& y, const float& z, const uint16_t& ring, const uint16_t& /*azimuth*/, const float& /*distance*/, const float& intensity) 
  {
    // convert polar coordinates to Euclidean XYZ
    VPoint point;
    point.ring = ring;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = intensity;

    // append this point to the cloud
    pc->points.push_back(point);
    ++pc->width;
  }
}

