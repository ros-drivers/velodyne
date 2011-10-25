/*
 *  Copyright (C) 2010 UT-Austin &  Austin Robot Technology,
 *  David Claridge, Michael Quinlan, Piyush Khandelwal, Jack O'Quin
 * 
 *  License: Modified BSD Software License 
 */

/** @file

 @brief This ROS node produces a point cloud containing
   all points that lie on an obstacle which is larger
   then HEIGHT_DIFF_THRESHOLD in height

Subscribes:

- @b velodyne/rawscan [velodyne::Data] raw data from one revolution of the velodyne

Publishes:

- @b veloydne/obstacles [sensor_msgs::PointCloud] points that lie on an obstacle
- @b veloydne/clear [sensor_msgs::PointCloud] points are clear of obstackes


@author David Claridge, Michael Quinlan 
*/


#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <velodyne/data.h>

#include <velodyne_height_map/height_map.h>

#define NODE "velodyne_height_map"

using namespace velodyne_common;

// local static data

namespace {
  velodyne::DataXYZ *data = NULL;
  
  HeightMap* height_map;

  sensor_msgs::PointCloud pc_;          // input PointCloud message
}

void preAllocatePointCloud(sensor_msgs::PointCloud *pc)
{
  // preallocate the anticipated amount of space for the point cloud
  pc->points.reserve(velodyne::SCANS_PER_REV);
  pc->channels.resize(1);
  pc->channels[0].name = "intensity";
  pc->channels[0].values.reserve(velodyne::SCANS_PER_REV);
}

void processXYZ(const std::vector<velodyne::laserscan_xyz_t> &scan)
{
  ros::Time stamp;
  std::string frame_id;
  data->getMsgHeaderFields(stamp, frame_id);
  pc_.header.stamp = stamp;
  pc_.header.frame_id = frame_id;

  // make sure there's enough space
  // (in case we're ever called with more data than expected)
  size_t numPoints = scan.size();
  pc_.points.resize(numPoints);
  pc_.channels[0].values.resize(numPoints);

  // copy all the data to the point cloud
  for (unsigned i = 0; i < numPoints; i++)
    {
      // Add this point to the cloud
      geometry_msgs::Point32 pt;      
      pt.x = scan[i].x;
      pt.y = scan[i].y;
      pt.z = scan[i].z;
      pc_.points[i] = pt;
      int intensity = scan[i].intensity;
      pc_.channels[0].values[i] = (float) intensity;
    }
  
  height_map->processData(pc_.points, stamp, frame_id);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, NODE);

  // create and initialize HeightMap instance
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");
  height_map = new HeightMap(node, private_nh);

  // allocate space to convert input data into a point cloud
  preAllocatePointCloud(&pc_);

  // allocate and initialize XYZ data input class
  data = new velodyne::DataXYZ();
  data->getParams();
  data->subscribeXYZ(processXYZ);
 
  if (0 != data->setup())
    return 2;
 
  // subscribe to velodyne input
  ros::Subscriber velodyne_scan =
    node.subscribe("velodyne/rawscan", 1,
                   &velodyne::Data::processRawScan,
                   (velodyne::Data *) data,
                   ros::TransportHints().tcpNoDelay(true));

  ROS_INFO(NODE ": starting main loop");

  ros::spin();                          // handle incoming data

  ROS_INFO(NODE ": exiting main loop");
  
  data->shutdown();
  delete data;

  return 0;
}
