/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Velodyne 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Velodyne LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
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

  /** @brief convert raw packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack(const velodyne_msgs::VelodynePacket &pkt, VPointCloud &pc)
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
            /*   Unpack a single (polar) laser scan in device frame.
             *
             *   pitch is relative to the plane of the unit (in its
             *     frame of reference): positive is above, negative is
             *     below.
             *
             *   heading is relative to the front of the unit (the
             *     outlet is the back): positive is clockwise, because
             *     the device rotates that direction about its Z axis.
             */
            float range;                ///< in meters
            float heading;              ///< in radians
            float pitch;                ///< in radians
            uint8_t laser_number;       ///< hardware laser number
            uint8_t intensity;          ///< unit-less intensity value

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
                pc.points.push_back(point);
                ++pc.width;
              }
          }
      }
  }  

} // namespace velodyne_rawdata
