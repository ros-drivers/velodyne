/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS nodelet transforms raw Velodyne HDL-64E 3D LIDAR data to a
    PointCloud in some frame of reference, typically "/odom".
*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>

#include <velodyne/data_xyz.h>
using namespace Velodyne;

class TransformNodelet: public nodelet::Nodelet
{
public:
  TransformNodelet()
  {
    data_ = new DataXYZ();
  }
  ~TransformNodelet()
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
  std::string frame_id_;                ///< target tf frame ID
  int npackets_;                        ///< number of packets to combine

  // point cloud buffer for a complete revolution
  sensor_msgs::PointCloudPtr pc_;
  unsigned pc_next_;
};

void TransformNodelet::onInit()
{
  // use private node handle to get parameters
  ros::NodeHandle private_nh = getPrivateNodeHandle();
  private_nh.param("frame_id", frame_id_, std::string("odom"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  frame_id_ = tf::resolve(tf_prefix, frame_id_);
  ROS_INFO_STREAM("target frame ID: " << frame_id_);
  private_nh.param("npackets", npackets_, PACKETS_PER_REV);

  data_->getParams();

  if (0 != data_->setup())
    return;

  // allocate space for the first PointCloud message
  allocSharedMsg();

  // subscribe to Velodyne data
  ros::NodeHandle node = getNodeHandle();
  velodyne_scan_ =
    data_->subscribe(node, "velodyne/packets", 10,
                     boost::bind(&TransformNodelet::processXYZ,
                                 this, _1, _2, _3),
                     ros::TransportHints().tcpNoDelay(true));

  output_ = node.advertise<sensor_msgs::PointCloud>("velodyne/pointcloud", 10);
}

/** \brief allocate space for shared PointCloud message
 *
 *  \post pc_ -> to message with enough space for one revolution
 *            of the device
 *        pc_next = 0
 */
void TransformNodelet::allocSharedMsg()
{
  // allocate a new shared pointer for zero-copy sharing with other nodelets
  pc_ = sensor_msgs::PointCloudPtr(new sensor_msgs::PointCloud);

  // allocate the anticipated amount of space for the point cloud
  pc_->points.resize(npackets_*SCANS_PER_PACKET);
  pc_->channels.resize(1);
  pc_->channels[0].name = "intensity";
  pc_->channels[0].values.resize(npackets_*SCANS_PER_PACKET);

  // set the exact point cloud size
  pc_->points.resize(npackets_*SCANS_PER_PACKET);
  pc_->channels[0].values.resize(npackets_*SCANS_PER_PACKET);
  pc_next_= 0;
}

/** \brief callback for XYZ points
 *
 *  converts Velodyne data for a single packet into a point cloud
 *  transforms the packet point cloud into the target frame
 *  collects transformed packets into a larger message (generally a full revolution)
 *  publishes those collected transformed data as a point cloud
 */
void TransformNodelet::processXYZ(const std::vector<laserscan_xyz_t> &scan,
                                   ros::Time stamp,
                                   const std::string &frame_id)
{
  if (output_.getNumSubscribers() == 0)         // no one listening?
    return;                                     // skip a lot of work

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
      pc_->header.frame_id = frame_id_; // target frame ID

      output_.publish(pc_);

      // nodelet sharing requires pc_ not to be modified after
      // publish(), so allocate a new message
      allocSharedMsg();
    }
}

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_common, TransformNodelet,
                        TransformNodelet, nodelet::Nodelet);
