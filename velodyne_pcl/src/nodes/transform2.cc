/*
 *  Copyright (C) 2009, 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS nodelet converts raw Velodyne HDL-64E 3D LIDAR packets to
    a PointCloud2 in the /odom frame.

    @author Jack O'Quin
    @author Jesse Vera
*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <velodyne/data_xyz.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <velodyne/ring_sequence.h>
#include <velodyne_pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>

namespace velodyne_pcl
{
  class Transform2: public nodelet::Nodelet
  {
  public:

    Transform2() {}
    ~Transform2() {}

  private:

    virtual void onInit();
    void processXYZ(const Velodyne::xyz_scans_t &scan,
                    ros::Time stamp,
                    const std::string &frame_id);

    /** in-line test whether a point is in range */
    bool pointInRange(const velodyne_pcl::PointXYZIR &point)
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

    // point cloud buffers for collecting points over time
    // (class members to avoid allocation and deallocation overhead)
    pcl::PointCloud<velodyne_pcl::PointXYZIR> pc_;
    int packetCount_;           ///< count of output packets collected
  };

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

    // allocate space for the output point cloud data
    pc_.points.reserve(config_.npackets*Velodyne::SCANS_PER_PACKET);
    pc_.points.clear();
    pc_.width = 0;
    pc_.height = 1;
    pc_.is_dense = true;
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
    
    // fill in point values
    size_t npoints = scan.size();
    for (size_t i = 0; i < npoints; ++i)
    {
      velodyne_pcl::PointXYZIR p;
      p.x = scan[i].x;
      p.y = scan[i].y;
      p.z = scan[i].z;
      if (pointInRange(p))
        {
          p.intensity = scan[i].intensity;
          p.ring = velodyne::LASER_RING[scan[i].laser_number];
          pc_.points.push_back(p);
          ++pc_.width;
        }
    }

    if (++packetCount_ >= config_.npackets)
      {
        // buffer is full, publish it
        sensor_msgs::PointCloud2Ptr outMsg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(pc_, *outMsg);

        // publish the accumulated, transformed point cloud
        outMsg->header.stamp = stamp;               // time of last packet
        outMsg->header.frame_id = frame_id;         // input frame ID
        //outMsg->header.frame_id = config_.frame_id; // target frame ID

        NODELET_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                             << " Velodyne points.");
        output_.publish(outMsg);
        pc_.points.clear();
        pc_.width = 0;
        packetCount_ = 0;
      }
  }

} // namespace velodyne_pcl


// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_pcl, Transform2,
                        velodyne_pcl::Transform2, nodelet::Nodelet);
