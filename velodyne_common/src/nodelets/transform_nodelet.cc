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
#include <geometry_msgs/Point32.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>

#include <velodyne/ring_sequence.h>
#include <velodyne/data_xyz.h>
using namespace Velodyne;               // use new data class interface

// channels to publish
namespace
{
  static const unsigned NCHANNELS = 3;
  static const std::string channel_name[NCHANNELS] =
    {
      "intensity",
      "ring",
      "heading"
    };
}

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

  /** in-line test whether a point is in range */
  bool pointInRange(geometry_msgs::Point32 &point)
  {
    float sum2 = (point.x * point.x 
                  + point.y * point.y
                  + point.z * point.z);
    return (sum2 >= min_range2_ && sum2 <= max_range2_);
  }

  // store squares of minimum and maximum ranges for efficient comparison
  float max_range2_;
  float min_range2_;

  DataXYZ *data_;
  ros::Subscriber velodyne_scan_;
  ros::Publisher output_;
  tf::TransformListener listener_;

  /// configuration parameters
  typedef struct {
    std::string frame_id;            ///< target tf frame ID
    double max_range;                ///< maximum range to publish
    double min_range;                ///< minimum range to publish
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

  private_nh.param("max_range", config_.max_range,
                   (double) Velodyne::DISTANCE_MAX);
  private_nh.param("min_range", config_.min_range, 2.0);
  NODELET_INFO_STREAM("data ranges to publish: ["
                      << config_.min_range << ", "
                      << config_.max_range << "]");
  min_range2_ = config_.min_range * config_.min_range;
  max_range2_ = config_.max_range * config_.max_range;

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
  msg.points.reserve(SCANS_PER_PACKET);
  msg.channels.resize(NCHANNELS);
  for (unsigned ch = 0; ch < NCHANNELS; ++ch)
    {
      msg.channels[ch].name = channel_name[ch];
      msg.channels[ch].values.reserve(SCANS_PER_PACKET);
    }
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
  outPtr_->channels.resize(NCHANNELS);
  for (unsigned ch = 0; ch < NCHANNELS; ++ch)
    {
      outPtr_->channels[ch].name = channel_name[ch];
      outPtr_->channels[ch].values.reserve(config_.npackets*SCANS_PER_PACKET);
    }
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

  // Clear working copies for transforming this packet.  These are
  // only class members to avoid reallocating them for every packet.
  inMsg_.points.clear();
  tfMsg_.points.clear();
  for (unsigned ch = 0; ch < NCHANNELS; ++ch)
    {
      inMsg_.channels[ch].values.clear();
      tfMsg_.channels[ch].values.clear();
    }

  // convert Velodyne data for this packet into a point cloud
  inMsg_.header.stamp = stamp;
  inMsg_.header.frame_id = frame_id;
  for (unsigned i = 0; i < scan.size(); ++i)
    {
      geometry_msgs::Point32 p;
      p.x = scan[i].x;
      p.y = scan[i].y;
      p.z = scan[i].z;
      if (pointInRange(p))
        {
          inMsg_.points.push_back(p);
          inMsg_.channels[0].values.push_back((float) scan[i].intensity);
          inMsg_.channels[1].values.push_back((float) velodyne::LASER_RING[scan[i].laser_number]);
          inMsg_.channels[2].values.push_back((float) scan[i].heading);
        }
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
      // only log tf error once every 20 times
      ROS_ERROR_THROTTLE(20, "%s", ex.what());
      return;                           // skip this packet
    }

  // append transformed packet data to end of output message
  outPtr_->points.insert(outPtr_->points.end(),
                         tfMsg_.points.begin(),
                         tfMsg_.points.end());
  for (unsigned ch = 0; ch < NCHANNELS; ++ch)
    {
      outPtr_->channels[ch].values.insert(outPtr_->channels[ch].values.end(),
                                          tfMsg_.channels[ch].values.begin(),
                                          tfMsg_.channels[ch].values.end());
    }

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
