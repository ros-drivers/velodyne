#ifndef __ORGANIZED_CLOUDXYZIR_H
#define __ORGANIZED_CLOUDXYZIR_H

#include <velodyne_pointcloud/datacontainerbase.h>

namespace velodyne_pointcloud
{
 class OrganizedCloudXYZIR : public velodyne_rawdata::DataContainerBase
  {
   public:

    OrganizedCloudXYZIR(
        const double max_range, const double min_range,
        const std::string& target_frame, const std::string& fixed_frame,
        const unsigned int num_lasers, const unsigned int scans_per_block);

    virtual void newLine();

    virtual void addPoint(
        const float& x,
        const float& y,
        const float& z,
        const uint16_t& ring,
        const uint16_t& azimuth,
        const float& distance,
        const float& intensity);

   private:
    std::map<uint16_t, velodyne_rawdata::VPoint*> organized_lasers;
  };
}
#endif //__ORGANIZED_CLOUDXYZIR_H

