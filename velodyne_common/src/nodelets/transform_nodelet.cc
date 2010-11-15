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
  void allocPacketMsg(sensor_msgs::PointCloud &msg);
  void allocSharedMsg();

  DataXYZ *data_;
  ros::Subscriber velodyne_scan_;
  ros::Publisher output_;
  tf::TransformListener listener_;

  /// configuration parameters
  typedef struct {
    std::string frame_id;            ///< target tf frame ID
    int npackets;                    ///< number of packets to combine
  } Config;
  Config config_;

  // point clouds data
  // (class members to avoid allocation and deallocation overhead)
  sensor_msgs::PointCloud inMsg_;       ///< input packet message
  sensor_msgs::PointCloud tfMsg_;       ///< transformed packet message
  sensor_msgs::PointCloudPtr outPtr_;   ///< output message shared pointer
  int packetCount_;                     ///< count of output packets collected
};

/** nodelet initialization */
void TransformNodelet::onInit()
{
  // use private node handle to get parameters
  ros::NodeHandle private_nh = getPrivateNodeHandle();
  private_nh.param("frame_id", config_.frame_id, std::string("odom"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);
  NODELET_INFO_STREAM("target frame ID: " << config_.frame_id);
  private_nh.param("npackets", config_.npackets, PACKETS_PER_REV);
  NODELET_INFO_STREAM("number of packets to accumulate: " << config_.npackets);

  data_->getParams();

  if (0 != data_->setup())
    return;

  // allocate space for the first PointCloud message
  allocSharedMsg();
  packetCount_ = 0;

  // allocate exact sizes for inMsg_ and tfMsg_ (single packet)
  allocPacketMsg(inMsg_);
  allocPacketMsg(tfMsg_);

  // advertise output point cloud (before subscribing to input data)
  ros::NodeHandle node = getNodeHandle();
  output_ = node.advertise<sensor_msgs::PointCloud>("velodyne/pointcloud", 10);

  // subscribe to Velodyne data
  velodyne_scan_ =
    data_->subscribe(node, "velodyne/packets", 10,
                     boost::bind(&TransformNodelet::processXYZ,
                                 this, _1, _2, _3),
                     ros::TransportHints().tcpNoDelay(true));
}

/** \brief allocate space in a single packet PointCloud message
 *
 *  \param msg message with enough space for one packet
 */
void TransformNodelet::allocPacketMsg(sensor_msgs::PointCloud &msg)
{
  msg.points.resize(SCANS_PER_PACKET);
  msg.channels.resize(1);
  msg.channels[0].name = "intensity";
  msg.channels[0].values.resize(SCANS_PER_PACKET);
}

/** \brief allocate space for shared PointCloud message
 *
 *  \post outPtr_ -> message with enough space reserved for the
 *                configured number of packets
 */
void TransformNodelet::allocSharedMsg()
{
  // allocate a new shared pointer for zero-copy sharing with other nodelets
  outPtr_ = sensor_msgs::PointCloudPtr(new sensor_msgs::PointCloud);

  // allocate the anticipated amount of space for the point cloud
  outPtr_->points.reserve(config_.npackets*SCANS_PER_PACKET);
  outPtr_->channels.resize(1);
  outPtr_->channels[0].name = "intensity";
  outPtr_->channels[0].values.reserve(config_.npackets*SCANS_PER_PACKET);

  // clear the contents (should already be done by constructor)
  outPtr_->points.clear();
  outPtr_->channels[0].values.clear();
}

/** \brief callback for packets in XYZ format
 *
 *  converts Velodyne data for a single packet into a point cloud
 *  transforms the packet point cloud into the target frame
 *  collects transformed packets into a larger message (generally a full revolution)
 *  periodically publishes those collected transformed data as a point cloud
 */
void TransformNodelet::processXYZ(const std::vector<laserscan_xyz_t> &scan,
                                   ros::Time stamp,
                                   const std::string &frame_id)
{
  if (output_.getNumSubscribers() == 0)         // no one listening?
    return;                                     // avoid much work

  // convert Velodyne data for this packet into a point cloud
  inMsg_.header.stamp = stamp;
  inMsg_.header.frame_id = frame_id;
  inMsg_.points.resize(scan.size());
  inMsg_.channels[0].values.resize(scan.size());
  for (unsigned i = 0; i < scan.size(); ++i)
    {
      inMsg_.points[i].x = scan[i].x;
      inMsg_.points[i].y = scan[i].y;
      inMsg_.points[i].z = scan[i].z;
      inMsg_.channels[0].values[i] = (float) scan[i].intensity;
    }

  // transform the packet point cloud into the target frame
  try
    {
      NODELET_DEBUG_STREAM("transforming from" << inMsg_.header.frame_id
                           << " to " << config_.frame_id);
      listener_.transformPointCloud(config_.frame_id, stamp,
                                    inMsg_, frame_id, tfMsg_);
    }
  catch (tf::TransformException ex)
    {
      ROS_ERROR_THROTTLE(5, "%s", ex.what());
      return;                           // skip this packet
    }

  // append transformed packet data to end of output message
  outPtr_->points.insert(outPtr_->points.end(),
                         tfMsg_.points.begin(),
                         tfMsg_.points.end());
  outPtr_->channels[0].values.insert(outPtr_->channels[0].values.end(),
                                     tfMsg_.channels[0].values.begin(),
                                     tfMsg_.channels[0].values.end());

  if (++packetCount_ >= config_.npackets)
    {
      // buffer is full, publish it
      NODELET_DEBUG_STREAM("Publishing " << outPtr_->points.size()
                           << " Velodyne points.");

      // publish the accumulated, transformed point cloud
      outPtr_->header.stamp = stamp;               // time of last packet
      outPtr_->header.frame_id = config_.frame_id; // target frame ID
      output_.publish(outPtr_);

      // nodelet sharing requires outPtr_ not to be modified after
      // publish(), so allocate a new message
      allocSharedMsg();
      packetCount_ = 0;
    }
}

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_common, TransformNodelet,
                        TransformNodelet, nodelet::Nodelet);
