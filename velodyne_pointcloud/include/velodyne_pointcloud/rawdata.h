/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Velodyne 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 */

#ifndef __VELODYNE_RAWDATA_H
#define __VELODYNE_RAWDATA_H

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/calibration.h>

namespace velodyne_rawdata
{
  // Shorthand typedefs for point cloud representations
  typedef velodyne_pointcloud::PointXYZIR VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  /**
   * Raw Velodyne packet constants and structures.
   */
  static const int SIZE_BLOCK = 100;
  static const int RAW_SCAN_SIZE = 3;
  static const int SCANS_PER_BLOCK = 32;
  static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

  static const float ROTATION_RESOLUTION = 0.01f; /**< degrees */
  static const uint16_t ROTATION_MAX_UNITS = 36000; /**< hundredths of degrees */

  /** According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
   *  valid packets with readings up to 130.0. */
  static const float DISTANCE_MAX = 130.0f;        /**< meters */
  static const float DISTANCE_RESOLUTION = 0.002f; /**< meters */
  static const float DISTANCE_MAX_UNITS = (DISTANCE_MAX
                                           / DISTANCE_RESOLUTION + 1.0);
  
  /** @todo make this work for both big and little-endian machines */
  static const uint16_t UPPER_BANK = 0xeeff;
  static const uint16_t LOWER_BANK = 0xddff;
  
  enum ReturnMode {
    UnknownMode   = 0x00,
    Strongest     = 0x37,
    Last          = 0x38,
    Dual          = 0x39
  };
  
  inline std::ostream& operator<<(std::ostream& os, ReturnMode mode)
  {
    switch (mode)
    {
    case Strongest:
      os << "strongest";  break;
    case Last:
      os << "last";       break;
    case Dual:
      os << "dual";       break;
    default:
      os << "[unknown return mode]";
    }
    
    return os;
  }
  
  enum SensorModel {
    UnknownModel  = 0x00,
    Hdl32e        = 0x21,
    Vlp16         = 0x22,
    Hdl64e
  };
  
  inline std::ostream& operator<<(std::ostream& os, SensorModel model)
  {
    switch (model)
    {
    case Hdl32e:
      os << "HDL-32E";  break;
    case Vlp16:
      os << "VLP-16";   break;
    case Hdl64e:
      os << "HDL-64E";  break;
    default:
      os << "[unknown sensor model]";
    }
    
    return os;
  }
   
  /** Special Defines for VLP16 support **/
  static const int    VLP16_FIRINGS_PER_BLOCK =   2;
  static const int    VLP16_SCANS_PER_FIRING  =  16;
  static const float  VLP16_BLOCK_TDURATION   = 110.592f;   // [µs]
  static const float  VLP16_DSR_TOFFSET       =   2.304f;   // [µs]
  static const float  VLP16_FIRING_TOFFSET    =  55.296f;   // [µs]
  

  /** \brief Raw Velodyne data block.
   *
   *  Each block contains data from either the upper or lower laser
   *  bank.  The device returns three times as many upper bank blocks.
   *
   *  use stdint.h types, so things work with both 64 and 32-bit machines
   */
  typedef struct raw_block
  {
    uint16_t header;        ///< UPPER_BANK or LOWER_BANK
    uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
    uint8_t  data[BLOCK_DATA_SIZE];
  } raw_block_t;

  /** used for unpacking the first two data bytes in a block
   *
   *  They are packed into the actual data stream misaligned.  I doubt
   *  this works on big endian machines.
   */
  union two_bytes
  {
    uint16_t uint;
    uint8_t  bytes[2];
  };

  static const int PACKET_SIZE = 1206;
  static const int BLOCKS_PER_PACKET = 12;
  static const int PACKET_STATUS_SIZE = 4;
  static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

  /** \brief Raw Velodyne packet.
   *
   *  revolution is described in the device manual as incrementing
   *    (mod 65536) for each physical turn of the device.  Our device
   *    seems to alternate between two different values every third
   *    packet.  One value increases, the other decreases.
   *
   *  \todo figure out if revolution is only present for one of the
   *  two types of status fields
   *
   *  status has either a temperature encoding or the microcode level
   */
  typedef struct raw_packet
  {
    raw_block_t blocks[BLOCKS_PER_PACKET];
    uint16_t revolution;
    uint8_t status[PACKET_STATUS_SIZE]; 
  } raw_packet_t;
  

  /** \brief Velodyne data conversion class */
  class RawData
  {
  public:

    RawData();
    ~RawData() {}

    /** \brief Set up for data processing.
     *
     *  Perform initializations needed before data processing can
     *  begin:
     *
     *    - read device-specific angles calibration
     *
     *  @param private_nh private node handle for ROS parameters
     *  @returns 0 if successful;
     *           errno value for failure
     */
    int setup(ros::NodeHandle private_nh);

    /** @brief convert raw packet to point cloud
     *
     *  @param pkt raw packet to unpack
     *  @param pc shared pointer to point cloud (points are appended)
     *  @param[out] m return mode of the laser scanner: last, strongest, or dual
     */
    void unpack(const velodyne_msgs::VelodynePacket &pkt, VPointCloud &pc);
    
    void setParameters(double min_range, double max_range, double view_direction,
                       double view_width);
    
    /// \brief Reads scanner's return mode from given packet's factory bytes.
    /// \note Supports only VLP-16 with firmware version 3.0.23.0 or higher.
    ReturnMode getReturnMode(const velodyne_msgs::VelodynePacket& pkt) const;
    
    /// \brief Reads the sensor model from number of lasers in calibration.
    SensorModel getSensorModel() const;
    
  private:

    /** configuration parameters */
    typedef struct {
      std::string calibrationFile;     ///< calibration file name
      double max_range;                ///< maximum range to publish
      double min_range;                ///< minimum range to publish
      int min_angle;                   ///< minimum angle to publish
      int max_angle;                   ///< maximum angle to publish
      
      double tmp_min_angle;
      double tmp_max_angle;
    } Config;
    Config config_;

    /** 
     * Calibration file
     */
    velodyne_pointcloud::Calibration calibration_;
    float sin_rot_table_[ROTATION_MAX_UNITS];
    float cos_rot_table_[ROTATION_MAX_UNITS];
    
    /** add private function to handle the VLP16 **/ 
    /** @brief convert raw VLP16 packet to point cloud
     *
     *  @param pkt raw packet to unpack
     *  @param pc shared pointer to point cloud (points are appended)
     */
    void unpack_vlp16(const velodyne_msgs::VelodynePacket &pkt, VPointCloud &pc);

    /** in-line test whether a point is in range */
    bool pointInRange(float range)
    {
      return (range >= config_.min_range
              && range <= config_.max_range);
    }
  };

} // namespace velodyne_rawdata

#endif // __VELODYNE_RAWDATA_H
