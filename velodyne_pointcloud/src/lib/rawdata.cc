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
 *  Velodyne 3D LIDAR data accessor base class.
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
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/ring_sequence.h>

namespace velodyne_rawdata
{
  ////////////////////////////////////////////////////////////////////////
  //
  // RawData base class implementation
  //
  ////////////////////////////////////////////////////////////////////////

  RawData::RawData()
  {
    memset(&upper_, 0, sizeof(upper_));
    memset(&lower_, 0, sizeof(lower_));
  }

#ifdef DEPRECATED_RAWDATA         // define DEPRECATED methods & types
  /** handle raw scan ROS topic message */
  void RawData::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    rawScan_ = scanMsg;              // save pointer to entire message

    // invoke callback for each packet
    for (unsigned i = 0; i < rawScan_->packets.size(); ++i)
      {
        processPacket(&rawScan_->packets[i], rawScan_->header.frame_id);
      }
  }
#endif // DEPRECATED_RAWDATA     // define DEPRECATED methods & types

  /** Set up for on-line operation. */
  int RawData::setup(ros::NodeHandle private_nh)
  {
    private_nh.param("max_range", config_.max_range,
                     (double) velodyne_rawdata::DISTANCE_MAX);
    private_nh.param("min_range", config_.min_range, 2.0);
    ROS_INFO_STREAM("data ranges to publish: ["
                    << config_.min_range << ", "
                    << config_.max_range << "]");

    // get path to angles.config file for this device
    if (!private_nh.getParam("angles", anglesFile_))
      {
        ROS_ERROR_STREAM("No calibration angles specified! (using test values)");

        // use velodyne_pointcloud test version as a default
        std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
        anglesFile_ = pkgPath + "/tests/angles.config";
      }

    ROS_INFO_STREAM("correction angles: " << anglesFile_);

    // read angles correction file for this specific unit
    std::ifstream config(anglesFile_.c_str());
    if (!config)
      {
        ROS_ERROR_STREAM("Failure opening Velodyne angles correction file: " 
                         << anglesFile_);
        return -1;
      }
  
    int index = 0;
    float rotational = 0;
    float vertical = 0;
    int enabled = 0;
    float offset1 = 0;
    float offset2 = 0;
    float offset3 = 0;
    float horzCorr = 0;
    float vertCorr = 0;
  
    correction_angles * angles = 0;
  
    char buffer[256];
    while(config.getline(buffer, sizeof(buffer)))
      {
        if (buffer[0] == '#') 
          continue;
        else if (strcmp(buffer, "upper") == 0)
          continue;
        else if (strcmp(buffer, "lower") == 0) 
          continue;
        else if ((sscanf(buffer,"%d %f %f %f %f %f %d", &index, &rotational,
                         &vertical, &offset1, &offset2,
                         &offset3, &enabled) == 7)
                 || (sscanf(buffer,"%d %f %f %f %f %f %f %f %d", &index,
                            &rotational, &vertical, &offset1, &offset2,
                            &offset3, &vertCorr, &horzCorr, &enabled) == 9))
          {
            int ind=index;
            if (index < 32)
              {
                angles=&lower_[0];
              }
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
            angles[ind].horzCorr = horzCorr;
            angles[ind].vertCorr = vertCorr;
            angles[ind].enabled = enabled;

// Nodes start with log level INFO by default, so it is hard to catch 
// this when using a ROS_DEBUG message, hence the #define DEBUG_ANGLES
//#define DEBUG_ANGLES 1    
#ifdef DEBUG_ANGLES
            ROS_INFO("%d %.2f %.6f %.f %.f %.2f %.3f %.3f %d",
                     index, rotational, vertical,
                     angles[ind].offset1,
                     angles[ind].offset2,
                     angles[ind].offset3,
                     angles[ind].horzCorr,
                     angles[ind].vertCorr,
                     angles[ind].enabled);
#endif
          }
      }

    config.close();
    return 0;
  }

  ////////////////////////////////////////////////////////////////////////
  //
  // RawDataScans class implementation
  //
  ////////////////////////////////////////////////////////////////////////

  RawDataScans::RawDataScans()
  {
#ifdef DEPRECATED_RAWDATA         // define DEPRECATED methods & types
    // reserve vector space before processing, we don't want to
    // reallocate in real time
    scans_.reserve(SCANS_PER_REV);
#endif // DEPRECATED_RAWDATA     // define DEPRECATED methods & types
  }

#ifdef DEPRECATED_RAWDATA         // define DEPRECATED methods & types
  /** \brief convert raw packet to laserscan format */
  void RawDataScans::packet2scans(const raw_packet_t *raw, laserscan_t *scans)
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

  /** \brief Process Velodyne packet. */
  void RawDataScans::processPacket(const velodyne_msgs::VelodynePacket *pkt,
                                   const std::string &frame_id)
  {
    // unpack scans from the raw packet
    scans_.resize(SCANS_PER_PACKET);
    packet2scans((raw_packet_t *) &pkt->data[0], &scans_[0]);

    // invoke the subscribed scans callback, if any
    if (cb_)
      cb_(scans_, pkt->stamp, frame_id);
  }
#endif // DEPRECATED_RAWDATA     // define DEPRECATED methods & types

  ////////////////////////////////////////////////////////////////////////
  //
  // RawDataXYZ class implementation
  //
  ////////////////////////////////////////////////////////////////////////

  RawDataXYZ::RawDataXYZ()
  {
#ifdef DEPRECATED_RAWDATA         // define DEPRECATED methods & types
    // reserve vector space before processing, we don't want to
    // reallocate in real time
    xyzScans_.reserve(SCANS_PER_REV);
#endif // DEPRECATED_RAWDATA     // define DEPRECATED methods & types
  }

#ifdef DEPRECATED_RAWDATA         // define DEPRECATED methods & types

  inline void RawDataXYZ::scan2xyz(const laserscan_t *scan,
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

  /** \brief Process a Velodyne packet message. */
  void RawDataXYZ::processPacket(const velodyne_msgs::VelodynePacket *pkt,
                                 const std::string &frame_id)
  {
    // run the base class method
    RawDataScans::processPacket(pkt, frame_id);

    // fill in xyzScan_ vector
    xyzScans_.resize(scans_.size());
    for (unsigned i = 0; i < xyzScans_.size(); i++)
      {
        scan2xyz(&scans_[i], &xyzScans_[i]);
      }

    // invoke the subscribed XYZ callback, if any
    if (cb_)
      cb_(xyzScans_, pkt->stamp, frame_id);
  }

#else  // new raw data methods

  /** @brief convert raw packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawDataXYZ::unpack(const velodyne_msgs::VelodynePacket &pkt,
                          VPointCloud::Ptr &pc)
  {
    ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

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
            float range;                ///< in meters
            float heading;              ///< in radians
            float pitch;                ///< in radians
            uint8_t  laser_number;
            uint8_t  intensity;

            laser_number = j + bank_origin;

            //if(!corrections[j].enabled) 
            //  do what???

            // beware: the Velodyne turns clockwise
            heading = 
              angles::normalize_angle(-(rotation - corrections[j].rotational));
            pitch   = corrections[j].vertical;
      
            union two_bytes tmp;
            tmp.bytes[0] = raw->blocks[i].data[k];
            tmp.bytes[1] = raw->blocks[i].data[k+1];

            // convert range to meters and apply quadratic correction
            range = tmp.uint * DISTANCE_RESOLUTION;
            range =
              (corrections[j].offset1 * range * range
               + corrections[j].offset2 * range
               + corrections[j].offset3);
      
            intensity = raw->blocks[i].data[k+2];

            if (pointInRange(range))
              {
                // convert polar coordinates to Euclidean XYZ
                VPoint point;
                float xy_projection = range * cosf(pitch);
                point.ring = velodyne_rawdata::LASER_RING[laser_number];
                point.x = xy_projection * cosf(heading);
                point.y = xy_projection * sinf(heading);
                point.z = range * sinf(pitch);
                point.intensity = intensity;

                // append this point to the cloud
                pc->points.push_back(point);
                ++pc->width;
              }
          }
      }
  }  

#endif // DEPRECATED_RAWDATA     // define DEPRECATED methods & types

} // namespace velodyne_pointcloud
