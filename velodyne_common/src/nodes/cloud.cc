/*
 *  Copyright (C) 2009 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node converts raw Velodyne HDL-64E 3D LIDAR data to a
    PointCloud.

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <velodyne/data_xyz.h>
using namespace Velodyne;

#define NODE "velodyne_cloud"

// command options
static int qDepth = 1;                  // ROS topic queue size

// local static data
static ros::Publisher output_;

// point cloud buffer for a complete revolution
sensor_msgs::PointCloudPtr pc_;
unsigned pc_next_;


/** \brief allocate space for shared PointCloud message
 *
 *  \post pc_ -> to message with enough space for one revolution
 *            of the device
 *        pc_next = 0
 */
void allocSharedMsg()
{
  // allocate a new shared pointer for zero-copy sharing with other nodelets
  pc_ = sensor_msgs::PointCloudPtr(new sensor_msgs::PointCloud);

  // allocate the anticipated amount of space for the point cloud
  pc_->points.resize(SCANS_PER_REV);
  pc_->channels.resize(1);
  pc_->channels[0].name = "intensity";
  pc_->channels[0].values.resize(SCANS_PER_REV);

  // set the exact point cloud size
  pc_->points.resize(SCANS_PER_REV);
  pc_->channels[0].values.resize(SCANS_PER_REV);
  pc_next_= 0;
}

/** \brief callback for XYZ points
 *
 * publishes Velodyne data points as a point cloud
 */
void processXYZ(const std::vector<laserscan_xyz_t> &scan,
                ros::Time stamp,
                const std::string &frame_id)
{

  // guard against vector indexing overflow
  ROS_ASSERT(pc_next_ + scan.size() <= pc_->points.size());

  for (unsigned i = 0; i < scan.size(); ++i)
    {
      pc_->points[pc_next_].x = scan[i].x;
      pc_->points[pc_next_].y = scan[i].y;
      pc_->points[pc_next_].z = scan[i].z;
      pc_->channels[0].values[pc_next_] = (float) scan[i].intensity;
      ++pc_next_;
    }

  if (pc_next_ == pc_->points.size())
    {
      // buffer is full, publish it
      ROS_DEBUG_STREAM("Publishing " << pc_->points.size()
                       << " Velodyne points.");

      // set time stamp and frame ID based on last packet received
      pc_->header.stamp = stamp;
      pc_->header.frame_id = frame_id;

      ROS_DEBUG_STREAM("Publishing " << pc_next_ << " Velodyne points.");
      output_.publish(pc_);

      // nodelet sharing requires pc_ not to be modified after
      // publish(), so allocate a new message
      allocSharedMsg();
    }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  // allocate space for the first PointCloud message
  allocSharedMsg();

  static DataXYZ *data = new DataXYZ();
  data->getParams();

  if (0 != data->setup())
    return 2;

  // subscribe to velodyne input -- make sure queue depth is minimal,
  // so any missed scans are discarded.  Otherwise latency gets out of
  // hand.  It's bad enough anyway.
  ros::Subscriber velodyne_scan =
    data->subscribe(node, "velodyne/packets", 1,
                     boost::bind(&processXYZ, _1, _2, _3),
                     ros::TransportHints().tcpNoDelay(true));

  output_ = node.advertise<sensor_msgs::PointCloud>("velodyne/pointcloud",
                                                   qDepth);

  ros::spin();                          // handle incoming data

  data->shutdown();
  delete data;

  return 0;
}
