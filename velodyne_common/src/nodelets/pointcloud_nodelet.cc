/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS nodelet converts raw Velodyne HDL-64E 3D LIDAR data to a
    PointCloud.

*/

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <velodyne/data_xyz.h>
using namespace Velodyne;

class PointCloudNodelet: public nodelet::Nodelet
{
public:
  PointCloudNodelet()
  {
    data_ = new DataXYZ();
  }
  ~PointCloudNodelet()
  {
    delete data_;
  }

private:
  virtual void onInit();
  void processXYZ(const std::vector<laserscan_xyz_t> &scan,
                  ros::Time stamp,
                  const std::string &frame_id);
  void allocSharedMsg();

  DataXYZ *data_;
  ros::Subscriber velodyne_scan_;
  ros::Publisher output_;

  // point cloud buffer for a complete revolution
  sensor_msgs::PointCloudPtr pc_;
  unsigned pc_next_;
};

void PointCloudNodelet::onInit()
{
  ros::NodeHandle node = getNodeHandle();

  // allocate space for the first PointCloud message
  allocSharedMsg();

  data_->getParams();

  if (0 != data_->setup())
    return;

  // subscribe to Velodyne data
  velodyne_scan_ =
    data_->subscribe(node, "velodyne/packets", 10,
                     boost::bind(&PointCloudNodelet::processXYZ,
                                 this, _1, _2, _3),
                     ros::TransportHints().tcpNoDelay(true));

  output_ = node.advertise<sensor_msgs::PointCloud>("velodyne/pointcloud", 1);
}

/** \brief allocate space for shared PointCloud message
 *
 *  \post pc_ -> to message with enough space for one revolution
 *            of the device
 *        pc_next = 0
 */
void PointCloudNodelet::allocSharedMsg()
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
 *  publishes Velodyne data points as a point cloud
 */
void PointCloudNodelet::processXYZ(const std::vector<laserscan_xyz_t> &scan,
                                   ros::Time stamp,
                                   const std::string &frame_id)
{
  //if (output_.getNumSubscribers() == 0)         // no one listening?
  //  return;

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
      NODELET_DEBUG_STREAM("Publishing " << pc_->points.size()
                           << " Velodyne points.");

      // set time stamp and frame ID based on last packet received
      pc_->header.stamp = stamp;
      pc_->header.frame_id = frame_id;

      output_.publish(pc_);

      // nodelet sharing requires pc_ not to be modified after
      // publish(), so allocate a new message
      allocSharedMsg();
    }
}

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_common, PointCloudNodelet,
                        PointCloudNodelet, nodelet::Nodelet);
