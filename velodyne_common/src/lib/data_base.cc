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
 *  Velodyne HDL-64E 3D LIDAR data accessor base class.
 *
 *  Base class for unpacking raw Velodyne LIDAR packets into various
 *  useful formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  \author Patrick Beeson
 *  \author Jack O'Quin
 */

#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>

#include <velodyne/data_base.h>

namespace Velodyne
{
  ////////////////////////////////////////////////////////////////////////
  //
  // Data base class implementation
  //
  ////////////////////////////////////////////////////////////////////////

  Data::Data(std::string ofile, std::string anglesFile)
  {
    ofile_ = ofile;
    anglesFile_ = anglesFile;

    ofp_ = NULL;
    memset(&upper_, 0, sizeof(upper_));
    memset(&lower_, 0, sizeof(lower_));
    getParams();
  }


  /** get ROS parameters
   *
   *  ROS parameter settings override constructor options
   *
   *
   * \returns 0, if successful;
   *          errno value, for failure
   */
  int Data::getParams()
  {
    // get parameters from "data" subordinate of private node handle
    ros::NodeHandle private_nh("~/data");

    private_nh.getParam("output", ofile_);

    // get path to angles.config file for this device
    if (!private_nh.getParam("angles", anglesFile_))
      {
        // use velodyne_common version as a default
        std::string pkgPath = ros::package::getPath("velodyne_common");
        anglesFile_ = pkgPath + "/etc/angles.config";
      }

    ROS_INFO_STREAM("correction angles: " << anglesFile_);

    return 0;
  }

  /** handle raw scan ROS topic message */
  void Data::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    rawScan_ = scanMsg;              // save pointer to entire message

    if (uninitialized_)
      return;

    // invoke callback for each packet
    for (unsigned i = 0; i < rawScan_->packets.size(); ++i)
      {
        processPacket(&rawScan_->packets[i], rawScan_->header.frame_id);
      }
  }

  /** Set up for on-line operation. */
  int Data::setup(void)
  {
    // open output file, if any
    ofp_ = NULL;
    if (ofile_ != "")
      {
        if (ofile_ == "-")              // using stdout?
          {
            ROS_INFO("output to stdout");
            ofp_ = stdout;
          }
        else                            // open named file
          {
            ROS_INFO("output file: %s", ofile_.c_str());
            ofp_ = fopen(ofile_.c_str(), "w");
            if (ofp_ == NULL)
              {
                int rc = errno;
                ROS_ERROR("failed to open \"%s\" (%s)",
                          ofile_.c_str(), strerror(rc));
                return rc;
              }
          }
      }

    // read angles correction file for this specific unit
    std::ifstream config(anglesFile_.c_str());
    if (!config)
      {
        std::cerr << "Failure opening Velodyne angles correction file: " 
                  << anglesFile_ << std::endl;
        return -1;
      }
  
    int index = 0;
    float rotational = 0;
    float vertical = 0;
    int enabled = 0;
    float offset1=0;
    float offset2=0;
    float offset3=0;
  
    correction_angles * angles = 0;
  
    char buffer[256];
    while(config.getline(buffer, sizeof(buffer)))
      {
        if (buffer[0] == '#') continue;
        else if (strcmp(buffer, "upper") == 0)
          continue;
        else if(strcmp(buffer, "lower") == 0) 
          continue;
        else if(sscanf(buffer,"%d %f %f %f %f %f %d", &index, &rotational,
                       &vertical, &offset1, &offset2, &offset3, &enabled) == 7)
          {
            int ind=index;
            if (index < 32) 
              angles=&lower_[0];
            else
              {
                angles=&upper_[0];
                ind=index-32;
              }
            angles[ind].rotational = angles::from_degrees(rotational);
            angles[ind].vertical   = angles::from_degrees(vertical);
            angles[ind].offset1 = offset1;
            angles[ind].offset2 = offset2;
            angles[ind].offset3 = offset3;
            angles[ind].enabled = enabled;

//#define DEBUG_ANGLES 1
#ifdef DEBUG_ANGLES
            ROS_DEBUG(stderr, "%d %.2f %.6f %.f %.f %.2f %d",
                      index, rotational, vertical,
                      angles[ind].offset1,
                      angles[ind].offset2,
                      angles[ind].offset3,
                      angles[ind].enabled);
#endif
          }
      }

    config.close();
    uninitialized_ = false;             // OK to start processing now
    return 0;
  }

} // namespace velodyne
