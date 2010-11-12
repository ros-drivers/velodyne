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
 *  Velodyne HDL-64E 3D LIDAR data accessor
 *
 *  Derived class for unpacking raw Velodyne LIDAR packets into
 *  a vector of individual laser scans.
 *
 *  \author Patrick Beeson
 *  \author Jack O'Quin
 */

#include <fstream>

#include <ros/ros.h>
#include <angles/angles.h>

#include <velodyne/data_scans.h>

namespace Velodyne
{
  ////////////////////////////////////////////////////////////////////////
  //
  // DataScans class implementation
  //
  ////////////////////////////////////////////////////////////////////////

  DataScans::DataScans(std::string ofile, std::string anglesFile):
    Data(ofile, anglesFile)
  {
    // reserve vector space before processing, we don't want to
    // reallocate in real time
    scans_.resize(SCANS_PER_REV);
    scansCB_ = NULL;
  }

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
  void DataScans::processRaw(const raw_packet_t *raw, size_t npackets)
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

    // invoke the subscribed scans callback, if any
    if (scansCB_)
      (*scansCB_)(scans_);
    else if (cb_)
      cb_(scans_);
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

} // namespace velodyne
