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
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

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
  struct
  {
    std::string frame_id;            ///< tf frame ID
    int    npackets;                 ///< number of packets to collect
    double rpm;                      ///< device rotation rate (RPMs)
  } config_;

  boost::shared_ptr<velodyne_driver::Input> input_;
  ros::Publisher output_;

  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
};
