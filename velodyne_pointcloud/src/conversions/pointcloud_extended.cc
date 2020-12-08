


#include <velodyne_pointcloud/pointcloud_extended.h>

namespace velodyne_pointcloud 
{
PointcloudExtended::PointcloudExtended(
    const double max_range, const double min_range,
    const std::string& target_frame, const std::string& fixed_frame,
    const unsigned int scans_per_block)
    : DataContainerBase(
        max_range, min_range, target_frame, fixed_frame,
        0, 1, true, scans_per_block, 12,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "azimuth", 1, sensor_msgs::PointField::UINT16,
        "distance", 1, sensor_msgs::PointField::FLOAT32,
        "time", 1, sensor_msgs::PointField::FLOAT32,
        "sub_segment", 1, sensor_msgs::PointField::UINT32,
        "rotation_segment", 1, sensor_msgs::PointField::UINT16,
        "firing_bin", 1, sensor_msgs::PointField::UINT16,
        "intensity", 1, sensor_msgs::PointField::UINT16,
        "ring", 1, sensor_msgs::PointField::UINT16,
        "laser_id", 1, sensor_msgs::PointField::UINT8),

        iter_x(cloud, "x"),
        iter_y(cloud, "y"),
        iter_z(cloud, "z"),
      iter_azimuth(cloud, "azimuth"),
        iter_distance(cloud, "distance"),
        iter_time(cloud, "time"),
        iter_sub_segment(cloud, "sub_segment"),
        iter_rotation_segment(cloud, "rotation_segment"),
        iter_firing_bin(cloud, "firing_bin"),
        iter_intensity(cloud, "intensity"),
        iter_ring(cloud, "ring"),
        iter_laser_id(cloud, "laser_id")

    {};

  void PointcloudExtended::setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg){
    DataContainerBase::setup(scan_msg);
    iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
    iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
    iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
    iter_azimuth = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud,"azimuth");
    iter_distance = sensor_msgs::PointCloud2Iterator<float>(cloud,"distance");
    iter_time = sensor_msgs::PointCloud2Iterator<float>(cloud,"time");
    iter_sub_segment = sensor_msgs::PointCloud2Iterator<uint32_t>(cloud,"sub_segment");
    iter_rotation_segment = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud,"rotation_segment");
    iter_firing_bin = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud,"firing_bin");
    iter_intensity = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud,"intensity");
    iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud,"ring");
    iter_laser_id = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud,"laser_id");




  }

  void PointcloudExtended::newLine()
  {}

void PointcloudExtended::addPoint(float x, float y, float z, const uint16_t ring,
                                  const uint16_t azimuth, const float distance,
                                  const float intensity, const float time)
{
  if(!pointInRange(distance)) return;

  // convert polar coordinates to Euclidean XYZ

  transformPoint(x, y, z);

  *iter_x = x;
  *iter_y = y;
  *iter_z = z;
  *iter_azimuth = azimuth;
  *iter_distance = distance;
  *iter_time = time;
  *iter_sub_segment = 0;
  *iter_rotation_segment = 0;
  *iter_firing_bin = 0;
  *iter_intensity = intensity;
  *iter_ring = ring;
  *iter_laser_id = 0;


  ++cloud.width;
  ++iter_x;
  ++iter_y;
  ++iter_z;
  ++iter_azimuth;
  ++iter_distance;
  ++iter_time;
  ++iter_sub_segment;
  ++iter_rotation_segment;
  ++iter_firing_bin;
  ++iter_intensity;
  ++iter_ring;
  ++iter_laser_id;
}

  void PointcloudExtended::addPoint(float x, float y, float z, const uint16_t ring,
                                    const uint16_t azimuth, const float distance,
                                    const float intensity, const float time,
                                    const uint32_t sub_segment, const uint16_t  rotation_segment,
                                    const uint16_t  firing_bin, const uint8_t laser_id)
  {
    if(!pointInRange(distance)) return;

    // convert polar coordinates to Euclidean XYZ

    transformPoint(x, y, z);

    *iter_x = x;
    *iter_y = y;
    *iter_z = z;
    *iter_azimuth = azimuth;
    *iter_distance = distance;
    *iter_time = time;
    *iter_sub_segment = sub_segment;
    *iter_rotation_segment = rotation_segment;
    *iter_firing_bin = firing_bin;
    *iter_intensity = intensity;
    *iter_ring = ring;
    *iter_laser_id = laser_id;


    ++cloud.width;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_azimuth;
    ++iter_distance;
    ++iter_time;
    ++iter_sub_segment;
    ++iter_rotation_segment;
    ++iter_firing_bin;
    ++iter_intensity;
    ++iter_ring;
    ++iter_laser_id;
  }
}

