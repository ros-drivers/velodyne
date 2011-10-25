/*
 *  Copyright (C) 2010 UT-Austin & Austin Robot Technology,
 *  David Claridge, Michael Quinlan, Jack O'Quin
 * 
 *  License: Modified BSD Software License 
 */

/** @file

   This ROS nodelet produces a point cloud containing all points that
   lie on an obstacle larger than HEIGHT_DIFF_THRESHOLD in height.

Subscribes:

- @b velodyne/pointcloud [sensor_msgs/PointCloud] raw data from one
     revolution of the Velodyne transformed to /odom frame of reference

Publishes:

- @b veloydne/obstacles [sensor_msgs::PointCloud] points that lie on an obstacle
- @b veloydne/clear [sensor_msgs::PointCloud] points are clear of obstackes


@author David Claridge, Michael Quinlan, Jack O'Quin
*/

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud.h>

#include <velodyne_height_map/height_map.h>


class HeightMapNodelet: public nodelet::Nodelet
{
public:

  HeightMapNodelet():
    height_map_(0)
  {}

  ~HeightMapNodelet()
  {
    if (height_map_)
      delete height_map_;
  }
  void onInit(void);

private:

  void processPointCloud(const sensor_msgs::PointCloudPtr &scan);

  ros::Subscriber velodyne_scan_;
  HeightMap* height_map_;
};

/** point cloud input callback */
void HeightMapNodelet::processPointCloud(const sensor_msgs::PointCloudPtr &scan)
{
  height_map_->processData(scan->points,
                           scan->header.stamp,
                           scan->header.frame_id);
}

void HeightMapNodelet::onInit(void)
{
  ros::NodeHandle node = getNodeHandle();
  ros::NodeHandle private_nh = getPrivateNodeHandle();
  height_map_ = new HeightMap(node, private_nh);
  
  // subscribe to velodyne input
  velodyne_scan_ =
    node.subscribe("velodyne/pointcloud", 1,
                   &HeightMapNodelet::processPointCloud, this,
                   ros::TransportHints().tcpNoDelay(true));
}

// Register this plugin with pluginlib.  Names must match height_map_nodelet.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_height_map, HeightMapNodelet,
                        HeightMapNodelet, nodelet::Nodelet);
