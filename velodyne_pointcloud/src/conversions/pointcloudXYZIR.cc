


#include <velodyne_pointcloud/pointcloudXYZIR.h>

namespace velodyne_pointcloud 
{

  PointcloudXYZIR::PointcloudXYZIR(
    const double max_range, const double min_range,
    const std::string& target_frame, const std::string& fixed_frame,
    const unsigned int scans_per_block)
    : DataContainerBase(
        max_range, min_range, target_frame, fixed_frame,
        0, 1, true, scans_per_block)
    {};

  void PointcloudXYZIR::newLine()
  {
    if(config_.transform)
    {
        computeTransformation(pcl_conversions::fromPCL(pc->header.stamp));
    }
  }

  void PointcloudXYZIR::addPoint(const float& x, const float& y, const float& z, const uint16_t& ring, const uint16_t& /*azimuth*/, const float& distance, const float& intensity)
  {
    if(!pointInRange(distance)) return;

    // convert polar coordinates to Euclidean XYZ
    velodyne_rawdata::VPoint point;
    point.ring = ring;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = intensity;

    if(config_.transform)
    {
      transformPoint(point);
    }

    // append this point to the cloud
    pc->points.push_back(point);
    ++pc->width;
  }
}

