

#ifndef __POINTCLOUDXYZIR_H
#define __POINTCLOUDXYZIR_H

#include <velodyne_pointcloud/datacontainerbase.h>

namespace velodyne_pointcloud 
{
 class PointcloudXYZIR : public velodyne_rawdata::DataContainerBase
  {
  public:

    PointcloudXYZIR(
        const double max_range, const double min_range,
        const std::string& target_frame, const std::string& fixed_frame,
        const unsigned int scans_per_block);

    virtual void newLine();

    virtual void addPoint(const float& x, const float& y, const float& z,
        const uint16_t& ring, const uint16_t& azimuth,
        const float& distance, const float& intensity);

  };
}
#endif //__POINTCLOUDXYZIR_H

