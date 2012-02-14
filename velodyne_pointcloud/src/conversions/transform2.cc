/*
 *  Copyright (C) 2009, 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS nodelet converts raw Velodyne 3D LIDAR packets to a
    PointCloud2 in the /odom frame.

    @author Jack O'Quin
    @author Jesse Vera
*/

/// @todo make sure all these includes are really necessary
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <velodyne/data_xyz.h>
#include <sensor_msgs/PointCloud2.h>

#include <velodyne_pointcloud/ring_sequence.h>
#include <velodyne_pointcloud/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

// include template implementations to transform a custom point cloud
#include <pcl_ros/impl/transforms.hpp>

/** types of point and cloud to work with */
typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

// instantiate template for transforming a VPointCloud
template bool
  pcl_ros::transformPointCloud<VPoint>(const std::string &,
                                       const VPointCloud &,
                                       VPointCloud &,
                                       const tf::TransformListener &);

namespace velodyne_pointcloud
{
  class Transform2: public nodelet::Nodelet
  {
  public:

    Transform2() {}
    ~Transform2() {}

    virtual void onInit();

  private:

    void allocPCloud(VPointCloud &pc, size_t reservation);
    void processXYZ(const Velodyne::xyz_scans_t &scan,
                    ros::Time stamp,
                    const std::string &frame_id);

    /** in-line test whether a point is in range */
    bool pointInRange(const velodyne_pointcloud::PointXYZIR &point)
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

    // Point cloud buffers for collecting points over time.  The inPc_
    // and tfPc_ are class members only to avoid allocation and
    // deallocation overhead.
    VPointCloud inPc_;              ///< input packet point cloud
    VPointCloud tfPc_;              ///< transformed packet point cloud
    VPointCloud outPc_;             ///< output point cloud
    int packetCount_;              ///< count of packets collected
  };

  /** @brief Allocate space in a point cloud. */
  void Transform2::allocPCloud(VPointCloud &pc, size_t reservation)
  {
    pc.points.reserve(reservation);
    pc.points.clear();
    pc.width = 0;
    pc.height = 1;
    pc.is_dense = true;
  }

  /** @brief Nodelet initialization. */
  void Transform2::onInit()
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
    NODELET_INFO_STREAM("number of packets to accumulate: "
                        << config_.npackets);

    data_->getParams();

    if (0 != data_->setup())
      return;

    // allocate space in packet-handling point clouds
    allocPCloud(inPc_, Velodyne::SCANS_PER_PACKET);
    allocPCloud(tfPc_, Velodyne::SCANS_PER_PACKET);
    allocPCloud(outPc_, Velodyne::SCANS_PER_PACKET*config_.npackets);
    packetCount_ = 0;

    // advertise output point cloud (before subscribing to input data)
    ros::NodeHandle node = getNodeHandle();
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne/pointcloud2", 10);

    // subscribe to Velodyne data
    velodyne_scan_ =
      data_->subscribe(node, "velodyne/packets", 10,
                       boost::bind(&Transform2::processXYZ,
                                   this, _1, _2, _3),
                       ros::TransportHints().tcpNoDelay(true));
  }

  /** \brief Callback for XYZ points.
   *
   *  Converts Velodyne data for a single packet into a point cloud.
   *  Transforms the packet point cloud into the target frame, and
   *  collects transformed packets into a larger message (generally a
   *  full revolution).  Periodically publishes those collected
   *  transformed data as a PointCloud2
   *
   *  @todo Maybe organize values into rows by ring number?  May not
   *        work, because some rings have more data than others (I
   *        think).
   */
  void Transform2::processXYZ(const Velodyne::xyz_scans_t &scan,
                                 ros::Time stamp,
                                 const std::string &frame_id)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    // clear input point cloud to handle this packet
    inPc_.points.clear();
    inPc_.width = 0;
    inPc_.header.stamp = stamp;
    inPc_.header.frame_id = frame_id;

    // fill in point values
    size_t npoints = scan.size();
    for (size_t i = 0; i < npoints; ++i)
    {
      velodyne_pointcloud::PointXYZIR p;
      p.x = scan[i].x;
      p.y = scan[i].y;
      p.z = scan[i].z;
      if (pointInRange(p))
        {
          p.intensity = scan[i].intensity;
          p.ring = velodyne::LASER_RING[scan[i].laser_number];
          inPc_.points.push_back(p);
          ++inPc_.width;
        }
    }

    // transform the packet point cloud into the target frame
    try
      {
        tfPc_.points.clear();           // is this needed?
        tfPc_.width = 0;
        NODELET_DEBUG_STREAM("transforming from" << inPc_.header.frame_id
                             << " to " << config_.frame_id);
        /// @todo wait for transform to be available
        listener_.waitForTransform(config_.frame_id, frame_id, stamp,
                                   ros::Duration(0.2));
        pcl_ros::transformPointCloud(config_.frame_id, inPc_, tfPc_,
                                     listener_);
      }
    catch (tf::TransformException ex)
      {
        // only log tf error once every 20 times
        ROS_WARN_THROTTLE(20, "%s", ex.what());
        return;                           // skip this packet
      }

    // append transformed packet data to end of output message
    outPc_.points.insert(outPc_.points.end(),
                         tfPc_.points.begin(),
                         tfPc_.points.end());
    outPc_.width += tfPc_.points.size();

    if (++packetCount_ >= config_.npackets)
      {
        // Output buffer full, allocate a new shared message pointer
        // to publish it.  That eliminates further copying when
        // nodelets in the same address space subscribe.
        sensor_msgs::PointCloud2Ptr outMsg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(outPc_, *outMsg);

        // publish the accumulated, transformed point cloud
        outMsg->header.stamp = stamp;               // time of last packet
        outMsg->header.frame_id = config_.frame_id; // target frame ID

        NODELET_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                             << " Velodyne points.");
        output_.publish(outMsg);
        outPc_.points.clear();
        outPc_.width = 0;
        packetCount_ = 0;
      }
  }

} // namespace velodyne_pointcloud

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_pointcloud, Transform2,
                        velodyne_pointcloud::Transform2, nodelet::Nodelet);
