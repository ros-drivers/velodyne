/*
 *  Copyright (C) 2009 Austin Robot Technology, Jack O'Quin
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

#include <velodyne/data.h>
#include <velodyne/running_stats.hh>

#define NODE "velodyne_cloud"

//using namespace velodyne_common;

class PointCloudNodelet: public nodelet::Nodelet
{
public:
  PointCloudNodelet()
  {
    data_ = new velodyne::DataXYZ();
  }
  ~PointCloudNodelet()
  {
    delete data_;
  }

private:
  virtual void onInit();
  void processXYZ(const std::vector<velodyne::laserscan_xyz_t> &scan);

  velodyne::DataXYZ *data_;
  ros::Subscriber velodyne_scan_;
  ros::Publisher output_;
};

void PointCloudNodelet::onInit()
{
  ros::NodeHandle node = getNodeHandle();

  data_->getParams();

  if (0 != data_->setup())
    return;

  // subscribe to velodyne input -- make sure queue depth is minimal,
  // so any missed scans are discarded.  Otherwise latency gets out of
  // hand.  It's bad enough anyway.
  velodyne_scan_ = data_->subscribe(node, "velodyne/rawscan", 1,
                                    boost::bind(&PointCloudNodelet::processXYZ, this, _1),
                                    ros::TransportHints().tcpNoDelay(true));

  output_ = node.advertise<sensor_msgs::PointCloud>("velodyne/pointcloud", 1);
}

/** \brief callback for XYZ points
 *
 * publishes Velodyne data points as a point cloud
 */
void PointCloudNodelet::processXYZ(const std::vector<velodyne::laserscan_xyz_t> &scan)
{
  if (output_.getNumSubscribers() == 0)         // no one listening?
    return;

  const roslib::Header *hdr = data_->getMsgHeader();

  // allocate a new shared pointer for zero-copy sharing with other nodelets
  sensor_msgs::PointCloudPtr pc(new sensor_msgs::PointCloud);

  // allocate the anticipated amount of space for the point cloud
  pc->points.resize(velodyne::SCANS_PER_REV);
  pc->channels.resize(1);
  pc->channels[0].name = "intensity";
  pc->channels[0].values.resize(velodyne::SCANS_PER_REV);

#if 0   // TODO add more channels (depending on parameters)
  pc->channels.resize(4);
  pc->channels[0].name = VC_RING;
  pc->channels[0].values.resize(velodyne::SCANS_PER_REV);
  pc->channels[1].name = VC_HEADING;
  pc->channels[1].values.resize(velodyne::SCANS_PER_REV);
  pc->channels[2].name = VC_INTENSITY;
  pc->channels[2].values.resize(velodyne::SCANS_PER_REV);
  pc->channels[3].name = VC_AVAILABLE;
  pc->channels[3].values.resize(velodyne::SCANS_PER_REV);
#endif

  // pass along original time stamp and frame ID
  pc->header.stamp = hdr->stamp;
  pc->header.frame_id = hdr->frame_id;

  // set the exact point cloud size -- the vectors should already have
  // enough space
  size_t npoints = scan.size();
  pc->points.resize(npoints);
  pc->channels[0].values.resize(npoints);

  for (unsigned i = 0; i < npoints; ++i)
    {
      pc->points[i].x = scan[i].x;
      pc->points[i].y = scan[i].y;
      pc->points[i].z = scan[i].z;
      pc->channels[0].values[i] = (float) scan[i].intensity;
    }

  // nodelet sharing requires pc not be modified after publish()ing
  NODELET_DEBUG_STREAM("Publishing " << npoints << " Velodyne points.");
  output_.publish(pc);
}

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.

// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_common, PointCloudNodelet,
                        PointCloudNodelet, nodelet::Nodelet);
