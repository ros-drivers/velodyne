

#ifndef __POINTCLOUDXYZIR_H
#define __POINTCLOUDXYZIR_H

#include <velodyne_pointcloud/datacontainerbase.h>

namespace velodyne_pointcloud 
{
 class PointcloudXYZIR : public velodyne_rawdata::DataContainerBase
  {
  public:

    virtual void newLine();

    virtual void addPoint(const float& x, const float& y, const float& z, const uint16_t& ring, const uint16_t& azimuth, const float& distance, const float& intensity);

  };
}
#endif //__POINTCLOUDXYZIR_H

