/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the Velodyne 3D LIDARs
 */

#include <string>
#include <ros/ros.h>
#include <velodyne_driver/input.h>

class VelodyneDriver
{
public:

  VelodyneDriver() {}
  ~VelodyneDriver() {}

  bool poll(void);
  void startup(ros::NodeHandle node,
               ros::NodeHandle private_nh);
  void shutdown(void);

private:

  // configuration parameters
  int npackets_;                        ///< number of packets to collect
  std::string frame_id_;                ///< tf frame ID

  boost::shared_ptr<velodyne_driver::Input> input_;
  ros::Publisher output_;
};
