/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009 Austin Robot Technology, Jack O'Quin
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
#include <ros/package.h>
#include <angles/angles.h>

#include <velodyne/data.h>

namespace velodyne
{
  ////////////////////////////////////////////////////////////////////////
  //
  // Data base class implementation
  //
  ////////////////////////////////////////////////////////////////////////

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
  void Data::processRawScan(const velodyne_common::RawScan::ConstPtr &raw_scan)
  {
    rawScan_ = raw_scan;                // save pointer to entire message
    velodyne::raw_packet_t *raw =
      (velodyne::raw_packet_t *) &raw_scan->data[0];
    size_t npackets = (raw_scan->data.size()
                       / velodyne_common::RawScan::PACKET_SIZE);
    processRaw(raw, npackets);
  }


  /** \brief Process raw Velodyne packets for an entire revolution. */
  void Data::processRaw(const velodyne::raw_packet_t *raw, size_t npackets)
  {
    if (rawCB_)
      (*rawCB_)(raw, npackets);
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

  ////////////////////////////////////////////////////////////////////////
  //
  // DataScans class implementation
  //
  ////////////////////////////////////////////////////////////////////////


  /** \brief convert raw packet to laserscan format */
  void DataScans::packet2scans(const raw_packet_t *raw, laserscan_t *scans)
  {
    int index = 0;                      // current scans entry
    uint16_t revolution = raw->revolution; // current revolution (mod 65536)

    for (int i = 0; i < BLOCKS_PER_PACKET; i++)
      {
        int bank_origin = 32;
        correction_angles *corrections = upper_;
        if (raw->blocks[i].header == LOWER_BANK)
          {
            bank_origin = 0;
            corrections = lower_;
          }

        float rotation = angles::from_degrees(raw->blocks[i].rotation
                                              * ROTATION_RESOLUTION);

        for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE)
          {
            scans[index].laser_number = j + bank_origin;

            //if(!corrections[j].enabled) 
            //  do what???

            // beware: the Velodyne turns clockwise
            /// \todo test against pdump2 with delayed normalize_angle()
            scans[index].heading = 
              angles::normalize_angle(-(rotation - corrections[j].rotational));
            scans[index].pitch   = corrections[j].vertical;
      
            union two_bytes tmp;
            tmp.bytes[0] = raw->blocks[i].data[k];
            tmp.bytes[1] = raw->blocks[i].data[k+1];

            // convert range to meters and apply quadratic correction
            scans[index].range = tmp.uint * DISTANCE_RESOLUTION;
            scans[index].range =
              (corrections[j].offset1 * scans[index].range * scans[index].range
               + corrections[j].offset2 * scans[index].range
               + corrections[j].offset3);
      
            scans[index].intensity = raw->blocks[i].data[k+2];
            scans[index].revolution = revolution;

            ++index;
          }
      }

    ROS_ASSERT(index == SCANS_PER_PACKET);
  }

  /** \brief Process raw Velodyne packets for an entire revolution. */
  void DataScans::processRaw(const velodyne::raw_packet_t *raw, size_t npackets)
  {
    if (uninitialized_)
      return;

    // run the base class method
    Data::processRaw(raw, npackets);

    // unpack scans from every packet
    scans_.resize(npackets * SCANS_PER_PACKET);
    for (unsigned i = 0; i < (unsigned) npackets; ++i)
      {
        packet2scans(&raw[i], &scans_[i * SCANS_PER_PACKET]);
      }

    if (!ros::ok())                     // check for ROS shutdown
      return;

    // invoke the subscribed XYZ callback, if any
    if (scansCB_)
      (*scansCB_)(scans_);
  }

  /** \brief print laser scan data
   *
   * This is not exactly the same format used by pcapconvert -t3, but
   * it's similar.
   *
   * \returns 0, if successful;
   *          errno value, for failure
   * \todo allow exact number of packets argument
   */
  int DataScans::print(void)
  {
    if (uninitialized_)                 // has setup() run?
      return EBADF;

    if (ofp_ == NULL)                   // no output desired?
      return 0;

    for (unsigned i = 0; i < scans_.size(); i++)
      {
        int rc = fprintf(ofp_, "%+08.6f %+08.6f %+011.6f %03u %02u %05u\n",
                         scans_[i].heading,
                         scans_[i].pitch,
                         scans_[i].range,
                         scans_[i].intensity,
                         scans_[i].laser_number,
                         scans_[i].revolution);
        if (rc < 0)
          return errno;

        // this loop runs for a long time, so check if a shutdown
        // signal was received
        if (!ros::ok())                 // ROS shutting down?
          return EINTR;                 // interrupted
      }

    return 0;
  }


  ////////////////////////////////////////////////////////////////////////
  //
  // DataXYZ class implementation
  //
  ////////////////////////////////////////////////////////////////////////

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
  void DataXYZ::processRaw(const velodyne::raw_packet_t *raw, size_t npackets)
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
