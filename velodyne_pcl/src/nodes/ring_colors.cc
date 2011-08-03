/*
 *  Copyright (C) 2009, 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS nodelet converts a Velodyne 3D LIDAR point cloud to PCL
    PointXYZRGB, assigning colors for visualization of the laser
    rings.

    @author Jack O'Quin
*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <velodyne/data_base.h>
#include <velodyne_pcl/point_types.h>

/// @todo make sure these includes are really necessary
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

/** types of point and cloud to work with */
typedef velodyne_pcl::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

namespace
{
  // RGB color values
  const int color_red =       0xff0000;
  const int color_orange =    0xff8800;
  const int color_yellow =    0xffff00;
  const int color_green =     0x00ff00;
  const int color_blue =      0x0000ff;
  const int color_violet =    0xff00ff;

  const int N_COLORS = 6;
  int rainbow[N_COLORS] =
    {color_red, color_orange, color_yellow,
     color_green, color_blue, color_violet};
}

namespace velodyne_pcl
{
  class RingColors: public nodelet::Nodelet
  {
  public:

    RingColors() {}
    ~RingColors() {}

    virtual void onInit();

  private:

    void convertPoints(const sensor_msgs::PointCloud2Ptr &inMsg);

    ros::Subscriber input_;
    ros::Publisher output_;

    // Point cloud buffers for converting points.  The inPc_ and
    // outPc_ are class members only to avoid allocation and
    // deallocation overhead.
    VPointCloud inPc_;              ///< input packet point cloud
    pcl::PointCloud<pcl::PointXYZRGB> outPc_; ///< output point cloud
  };

  /** @brief Nodelet initialization. */
  void RingColors::onInit()
  {
    // allocate space in packet-handling point clouds
    inPc_.points.reserve(Velodyne::SCANS_PER_PACKET);
    inPc_.points.clear();
    inPc_.width = 0;
    inPc_.height = 1;
    inPc_.is_dense = true;
    
    // allocate space in packet-handling point clouds
    outPc_.points.reserve(Velodyne::SCANS_PER_PACKET);
    outPc_.points.clear();
    outPc_.width = 0;
    outPc_.height = 1;
    outPc_.is_dense = true;

    // advertise output point cloud (before subscribing to input data)
    ros::NodeHandle node = getNodeHandle();
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne/rings", 10);

    // subscribe to Velodyne input cloud
    input_ =
      node.subscribe("velodyne/pointcloud2", 1,
                     &RingColors::convertPoints, this,
                     ros::TransportHints().tcpNoDelay(true));
  }

  /** \brief Callback for Velodyne XYZ points.
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
  void RingColors::convertPoints(const sensor_msgs::PointCloud2Ptr &inMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // do nothing
#if 0
    // clear input point cloud to handle this packet
    inPc_.points.clear();
    inPc_.width = 0;
    inPc_.header.stamp = inMsg->header.stamp;
    inPc_.header.frame_id = inMsg->header.frame_id;
#endif

    // convert ROS PointCloud2 message to PCL cloud
    pcl::fromROSMsg(*inMsg, inPc_);

    // convert point values
    outPc_.points.clear();
    outPc_.width = 0;
    size_t npoints = inPc_.points.size();
    for (size_t i = 0; i < npoints; ++i)
    {
      pcl::PointXYZRGB p;
      p.x = inPc_.points[i].x;
      p.y = inPc_.points[i].y;
      p.z = inPc_.points[i].z;

      // color lasers with the rainbow array
      int color = inPc_.points[i].ring % N_COLORS;
      p.rgb = *reinterpret_cast<float*>(&rainbow[color]);

      outPc_.points.push_back(p);
      ++outPc_.width;
    }

    // Allocate a new shared message pointer and publish the converted
    // output.  That eliminates further copying when nodelets in the
    // same address space subscribe.
    sensor_msgs::PointCloud2Ptr outMsg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(outPc_, *outMsg);
    outMsg->header.stamp = inMsg->header.stamp;
    outMsg->header.frame_id = inMsg->header.frame_id;
    NODELET_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                         << " Velodyne points.");
    output_.publish(outMsg);
  }

} // namespace velodyne_pcl

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_pcl, RingColors,
                        velodyne_pcl::RingColors, nodelet::Nodelet);
