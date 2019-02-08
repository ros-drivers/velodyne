

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

    virtual void setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg);

    virtual void addPoint(float x, float y, float z,
        uint16_t ring, uint16_t azimuth, float distance, float intensity);

    sensor_msgs::PointCloud2Iterator<float> iter_x, iter_y, iter_z, iter_intensity;
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring;

  };
}
#endif //__POINTCLOUDXYZIR_H

