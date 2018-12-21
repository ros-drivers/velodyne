


#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/pointcloudXYZIRR.h>

namespace velodyne_pointcloud 
{
  void PointcloudXYZIRR::addPoint(const float& x, const float& y, const float& z,
      const uint16_t& ring, const uint16_t& /*azimuth*/, const float& /*distance*/,
      const float& intensity, const uint8_t& return_type)
  {
    // convert polar coordinates to Euclidean XYZ
    velodyne_rawdata::VPoint point;
    point.ring = ring;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = intensity;
    point.return_type = return_type;

    // append this point to the cloud
    pc->points.push_back(point);
    ++pc->width;
  }
}

