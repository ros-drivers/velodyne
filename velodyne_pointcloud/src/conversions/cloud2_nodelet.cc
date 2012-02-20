/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This ROS nodelet converts raw Velodyne 3D LIDAR packets to a
    PointCloud2.

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "convert.h"

namespace velodyne_pointcloud
{
  class Cloud2Nodelet: public nodelet::Nodelet
  {
  public:

    Cloud2Nodelet() {}
    ~Cloud2Nodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Convert> conv_;
  };

  /** @brief Nodelet initialization. */
  void Cloud2Nodelet::onInit()
  {
    conv_.reset(new Convert(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace velodyne_pointcloud


// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_pointcloud, Cloud2Nodelet,
                        velodyne_pointcloud::Cloud2Nodelet, nodelet::Nodelet);
