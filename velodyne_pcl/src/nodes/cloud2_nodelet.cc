/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: base_cloud2_nodelet.cc 779 2010-11-13 19:41:40Z jesse.vera $
 */

/** \file

    This ROS nodelet converts raw Velodyne HDL-64E 3D LIDAR packets to
    a PointCloud2 in the /odom frame.

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <velodyne/data_xyz.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>

namespace velodyne_pcl
{

class Cloud2Nodelet: public nodelet::Nodelet
{
public:

  Cloud2Nodelet() {}
  ~Cloud2Nodelet() {}

private:

  virtual void onInit();
  void processXYZ(const std::vector<Velodyne::laserscan_xyz_t> &scan,
                  ros::Time stamp,
                  const std::string &frame_id);
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

  boost::shared_ptr<Velodyne::DataXYZ> data_;
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

#define USE_NATIVE_POINT_TYPE 0         // not working yet
#if USE_NATIVE_POINT_TYPE
  //PointType def
  struct VelodynePoint
  {
    PCL_ADD_POINT4D;          // preferred way of adding a XYZ+padding
    double time_stamp;
    uint8_t intensity;
    uint8_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
  } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

  POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePoint,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (uint8_t, intensity, intensity)
                                    (uint8_t, ring, ring));
#else
  // first, get things working with this built-in PCL type
  typedef pcl::PointXYZI VelodynePoint;
#endif

  // point cloud buffers for collecting points over time
  pcl::PointCloud<pcl::PointXYZI> pc_;
  sensor_msgs::PointCloud2Ptr outPtr_;  // ROS output message pointer
  //unsigned pc_next_;
};

void Cloud2Nodelet::onInit()
{
  data_.reset(new Velodyne::DataXYZ());

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

  private_nh.param("npackets", config_.npackets, Velodyne::PACKETS_PER_REV);
  NODELET_INFO_STREAM("number of packets to accumulate: " << config_.npackets);

  data_->getParams();

  if (0 != data_->setup())
    return;

  // allocate space for the first PointCloud2 message
  allocSharedMsg();

#if 0
  packetCount_ = 0;

  // allocate exact sizes for inMsg_ and tfMsg_ (single packet)
  allocPacketMsg(inMsg_);
  allocPacketMsg(tfMsg_);
#endif

  // advertise output point cloud (before subscribing to input data)
  ros::NodeHandle node = getNodeHandle();
  output_ =
    node.advertise<sensor_msgs::PointCloud2>("velodyne/pointcloud2", 10);

  // subscribe to Velodyne data
  velodyne_scan_ =
    data_->subscribe(node, "velodyne/packets", 10,
                     boost::bind(&Cloud2Nodelet::processXYZ,
                                 this, _1, _2, _3),
                     ros::TransportHints().tcpNoDelay(true));
}

/** \brief allocate space for shared PointCloud message
 *
 *  \post outPtr_ -> to message with enough space for one revolution
 *            of the device
 *        pc_next = 0
 */
void Cloud2Nodelet::allocSharedMsg()
{  
  // allocate a new shared pointer for zero-copy sharing with other nodelets
  outPtr_.reset(new sensor_msgs::PointCloud2());

/*
  // allocate the anticipated amount of space for the point cloud
  outPtr_->points.resize(SCANS_PER_REV);
  outPtr_->channels.resize(1);
  outPtr_->channels[0].name = "intensity";
  outPtr_->channels[0].values.resize(SCANS_PER_REV);

  // set the exact point cloud size
  outPtr_->points.resize(SCANS_PER_REV);
  outPtr_->channels[0].values.resize(SCANS_PER_REV);
  pc_next_= 0;
*/
}

/** \brief callback for XYZ points
 *
 *  Converts Velodyne data for a single packet into a point cloud.
 *  Transforms the packet point cloud into the target frame, and
 *  collects transformed packets into a larger message (generally a
 *  full revolution).  Periodically publishes those collected
 *  transformed data as a PointCloud2
 */
void
  Cloud2Nodelet::processXYZ(const std::vector<Velodyne::laserscan_xyz_t> &scan,
                            ros::Time stamp,
                            const std::string &frame_id)
{
  if (output_.getNumSubscribers() == 0)         // no one listening?
    return;                                     // avoid much work
  
  // guard against vector indexing overflow
  //ROS_ASSERT(pc_next_ + scan.size() <= outPtr_->points.size());

  // set the exact point cloud size
  size_t npoints = scan.size();
  pc_.points.resize(npoints);

  // (Maybe) organize values into rows by ring number?  May not work,
  // because some rings have more data than others (I think).
  pc_.width = npoints;
  pc_.height = 1;
  pc_.is_dense = true;

  // fill in point values
  for (size_t i = 0; i < npoints; ++i)
  {
    pc_.points[i].x = scan[i].x;
    pc_.points[i].y = scan[i].y;
    pc_.points[i].z = scan[i].z;
    pc_.points[i].intensity = scan[i].intensity;

#if USE_NATIVE_POINT_TYPE
    pc_[i].ring = velodyne::LASER_RING[scan[i].laser_number];
#endif
  }
  
  pcl::toROSMsg(pc_, *outPtr_);
 
  // set time stamp and frame ID based on last packet received
  outPtr_->header.stamp = stamp;
  outPtr_->header.frame_id = frame_id;
  
#if 1 // not yet accumulating multiple packets
  output_.publish(outPtr_);
  // nodelet sharing requires outPtr_ not to be modified after
  // publish(), so allocate a new message
  allocSharedMsg();
#else // accumulate multiple packets
  if (pc_next_ == outPtr_->points.size())
    {
      // buffer is full, publish it
      NODELET_DEBUG_STREAM("Publishing " << outPtr_->points.size()
                           << " Velodyne points.");

      // set time stamp and frame ID based on last packet received
      outPtr_->header.stamp = stamp;
      outPtr_->header.frame_id = frame_id;

      output_.publish(outPtr_);

      // nodelet sharing requires outPtr_ not to be modified after
      // publish(), so allocate a new message
      allocSharedMsg();
    }
#endif
}

} // namespace velodyne_pcl

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_pcl, Cloud2Nodelet,
                        velodyne_pcl::Cloud2Nodelet, nodelet::Nodelet);
