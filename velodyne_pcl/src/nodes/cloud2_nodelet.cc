/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
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

using namespace Velodyne;

class Cloud2Nodelet: public nodelet::Nodelet
{
public:

  Cloud2Nodelet()
  {
    data_ = new DataXYZ();
  }
  ~Cloud2Nodelet()
  {
    delete data_;
  }

private:

  virtual void onInit();
  void processXYZ(const std::vector<laserscan_xyz_t> &scan,
                  ros::Time stamp,
                  const std::string &frame_id);
  void allocSharedMsg();

  DataXYZ *data_;                       /// @todo use boost::shared_ptr
  ros::Subscriber velodyne_scan_;
  ros::Publisher output_;

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

  // point cloud buffer for a collection of points over time
  sensor_msgs::PointCloud2Ptr outPtr_;
  //PointCloud<VelodynePoint>::Ptr basePc_;
  //unsigned pc_next_;
};

void Cloud2Nodelet::onInit()
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
                     boost::bind(&Cloud2Nodelet::processXYZ,
                                 this, _1, _2, _3),
                     ros::TransportHints().tcpNoDelay(true));

  output_ =
    node.advertise<sensor_msgs::PointCloud2>("velodyne/pointcloud2", 1);
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
  outPtr_ = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2);
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
 *   publishes Velodyne data points as a PointCloud
 */
void Cloud2Nodelet::processXYZ(const std::vector<laserscan_xyz_t> &scan,
                                   ros::Time stamp,
                                   const std::string &frame_id)
{
  //if (output_.getNumSubscribers() == 0)         // no one listening?
  //  return;
  
  // guard against vector indexing overflow
  //ROS_ASSERT(pc_next_ + scan.size() <= outPtr_->points.size());
  
  pcl::PointCloud<VelodynePoint> cloud;

  // set the exact point cloud size -- the vectors should already have
  // enough space
  size_t npoints = scan.size();
  cloud.resize(npoints);

  for (int i = 0; i < npoints; ++i)
  {
    cloud.x[i] = scan[i].x;
    cloud->points[i].y = scan[i].y;
    cloud->points[i].z = scan[i].z;
    cloud->points[i].intensity = scan[i].intensity;
#if USE_NATIVE_POINT_TYPE
    cloud->points[i].ring = velodyne::LASER_RING[scan[i].laser_number];
    cloud->points[i].time_stamp = stamp;    
#endif
  }
  
  pcl::toROSMsg(*cloud, *outPtr_);
  
  // set time stamp and frame ID based on last packet received
  outPtr_->header.stamp = stamp;
  outPtr_->header.frame_id = frame_id;
  
  output_.publish(outPtr_);
  // nodelet sharing requires outPtr_ not to be modified after
  // publish(), so allocate a new message
  allocSharedMsg();
/*
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
*/
}

} // namespace velodyne_pcl

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_pcl, Cloud2Nodelet,
                        velodyne_pcl::Cloud2Nodelet, nodelet::Nodelet);
