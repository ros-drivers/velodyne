/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  \file
 *
 *  Classes for unpacking Velodyne HDL-64E 3D LIDAR data into several
 *  formats.
 *
 *  These classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provides it in several different formats
 *  for either on-line or off-line processing.
 *
 *  \author Patrick Beeson
 *  \author Jack O'Quin
 */

#include <fstream>
#include <ros/ros.h>
#include <velodyne/data_xyz.h>

namespace Velodyne
{
  ////////////////////////////////////////////////////////////////////////
  //
  // DataXYZ class implementation
  //
  ////////////////////////////////////////////////////////////////////////

  DataXYZ::DataXYZ(std::string ofile, std::string anglesFile):
    DataScans(ofile, anglesFile)
  {
    // reserve vector space before processing, we don't want to
    // reallocate in real time
    xyzScans_.resize(SCANS_PER_REV);
    xyzCB_ = NULL;
  }

  inline void DataXYZ::scan2xyz(const laserscan_t *scan,
                                laserscan_xyz_t *point)
  {
    float xy_projection = scan->range * cosf(scan->pitch);
    point->laser_number = scan->laser_number;
    point->heading = scan->heading;
    point->revolution = scan->revolution;
    point->x = xy_projection * cosf(scan->heading);
    point->y = xy_projection * sinf(scan->heading);
    point->z = scan->range * sinf(scan->pitch);
    point->intensity = scan->intensity;
  }

  /** \brief Process raw Velodyne packets for an entire revolution. */
  void DataXYZ::processRaw(const raw_packet_t *raw, size_t npackets)
  {
    if (uninitialized_)
      return;

    // run the base class method
    DataScans::processRaw(raw, npackets);

    // fill in xyzScan_ vector
    xyzScans_.resize(scans_.size());
    for (unsigned i = 0; i < xyzScans_.size(); i++)
      {
        scan2xyz(&scans_[i], &xyzScans_[i]);
      }

    if (!ros::ok())                     // check for ROS shutdown
      return;

    // invoke the subscribed XYZ callback, if any
    if (xyzCB_)
      (*xyzCB_)(xyzScans_);
    else if (cb_)
      cb_(xyzScans_);
  }


  /** \brief print laser scans in XYZ format
   *
   * \returns 0, if successful;
   *          errno value, for failure
   * \todo allow exact number of packets argument
   */
  int DataXYZ::print(void)
  {
    if (uninitialized_)                 // has setup() run?
      return EBADF;

    if (ofp_ == NULL)                   // no output desired?
      return 0;

    for (unsigned i = 0; i < xyzScans_.size(); i++)
      {
        // this leading zeros format is useful for test validation
        // TODO: (maybe) remove excess characters eventually
        int rc = fprintf(ofp_, "%02u %+08.6f %+011.6f %+011.6f %+011.6f %03u\n",
                         xyzScans_[i].laser_number,
                         xyzScans_[i].heading,
                         xyzScans_[i].x, xyzScans_[i].y, xyzScans_[i].z,
                         xyzScans_[i].intensity);
        if (rc < 0)
          return errno;

        // this loop runs for a long time, so check if a shutdown
        // signal was received
        if (!ros::ok())                 // ROS shutting down?
          return EINTR;                 // interrupted
      }

    return 0;
  }

} // namespace velodyne
