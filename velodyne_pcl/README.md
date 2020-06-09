## PointCloud2 to PCL PointCloud Conversion

Convert `sensor_msgs::PointCloud2` objects coming from the velodyne_pointcloud driver to `pcl::PointCloud<velodyne_pcl::PointXYZIRT>` objects.

The `sensor_msgs::PointCloud2` ROS message is constructed by using the `PointcloudXYZIRT`, or the `OrganizedCloudXYZIRT` container.
Both define the following channels:

* x - The x coord in Cartesian coordinates
* y - The y coord in Cartesian coordinates
* z - The z coord in Cartesian coordinates
* intensity - The measured intensity at the point
* ring - The ring number of the laser
* time - The time stamp of the measured point

To convert a `sensor_msgs::PointCloud2` published by the velodyne driver to PCL you could use the following code:

```
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

void cloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloud){

  pcl::PCLPointCloud2 pcl_pointcloud2;
  pcl_conversions::toPCL(*cloud, pcl_pointcloud2);
  pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr pcl_cloud_ptr(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
  pcl::fromPCLPointCloud2(pcl_pointcloud2, *pcl_cloud_ptr);

  // use pcl_cloud_ptr
}
```
