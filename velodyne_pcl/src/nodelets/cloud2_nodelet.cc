/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS nodelet converts raw Velodyne HDL-64E 3D LIDAR packets to
    a PointCloud2.

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <velodyne/data_xyz.h>
#include <sensor_msgs/PointCloud2.h>

#include <velodyne/ring_sequence.h>
#include <velodyne_pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

namespace velodyne_pcl
{
  class Cloud2Nodelet: public nodelet::Nodelet
  {
  public:

    Cloud2Nodelet() {}
    ~Cloud2Nodelet() {}

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

    /// configuration parameters
    typedef struct {
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
  void Cloud2Nodelet::onInit()
  {
    data_.reset(new Velodyne::DataXYZ());

    // use private node handle to get parameters
    ros::NodeHandle private_nh = getPrivateNodeHandle();

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
                       boost::bind(&Cloud2Nodelet::processXYZ,
                                   this, _1, _2, _3),
                       ros::TransportHints().tcpNoDelay(true));
  }

  /** \brief Callback for XYZ points.
   *
   *  Converts Velodyne data for a single packet into a point cloud.
   *  Collects packets into a larger message (generally a full
   *  revolution).  Periodically publishes collected data as a
   *  PointCloud2 message.
   */
  void Cloud2Nodelet::processXYZ(const Velodyne::xyz_scans_t &scan,
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

        // publish the accumulated point cloud
        outMsg->header.stamp = stamp;               // time of last packet
        outMsg->header.frame_id = frame_id;         // input frame ID

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
PLUGINLIB_DECLARE_CLASS(velodyne_pcl, Cloud2Nodelet,
                        velodyne_pcl::Cloud2Nodelet, nodelet::Nodelet);
