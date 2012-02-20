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
 *  Velodyne 3D LIDAR raw data accessor classes.
 *
 *  Classes for unpacking raw Velodyne LIDAR packets into various
 *  useful formats.
 *
 *  @note These interfaces are @b not exported from the @c
 *        velodyne_pointcloud package, but if needed they could be.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 */

#ifndef __VELODYNE_RAWDATA_H
#define __VELODYNE_RAWDATA_H

//#define DEPRECATED_RAWDATA 1      // define DEPRECATED methods & types

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/point_types.h>

namespace velodyne_rawdata
{
  // Shorthand typedefs for point cloud representations
  typedef velodyne_pointcloud::PointPolarIR VPolar;
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
  static const float ROTATION_MAX_UNITS = 36000; /**< hundredths of degrees */

  /** According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
   *  valid packets with readings up to 130.0. */
  static const float DISTANCE_MAX = 130.0f;        /**< meters */
  static const float DISTANCE_RESOLUTION = 0.002f; /**< meters */
  static const float DISTANCE_MAX_UNITS = (DISTANCE_MAX
                                           / DISTANCE_RESOLUTION + 1.0);
  static const uint16_t UPPER_BANK = 0xeeff;
  static const uint16_t LOWER_BANK = 0xddff;

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
  static const int PACKETS_PER_REV = 260;
  static const int SCANS_PER_REV = (SCANS_PER_PACKET * PACKETS_PER_REV);

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

  /** \brief Correction angles for a specific HDL-64E device. */
  struct correction_angles
  {
    float rotational;
    float vertical;
    float offset1, offset2, offset3;
    float horzCorr, vertCorr;
    int   enabled;
  };

  /** \brief Base Velodyne data class -- not used directly. */
  class RawData
  {
  public:

    RawData();
    ~RawData() {}

#ifdef DEPRECATED_RAWDATA         // define DEPRECATED methods & types
    /** \brief handle ROS topic message
     *
     *  This is the main entry point for handling ROS messages from
     *  the "velodyne/packets" topic, published by the driver nodelet.
     *  This is \b not a virtual function; derived classes may replace
     *  processPacket(), instead.
     *
     *  By default, the driver publishes packets for a complete
     *  rotation of the device in each message, but that is not
     *  guaranteed.
     */
    void processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);

    /** \brief Process a Velodyne packet.
     *
     * \param pkt -> VelodynePacket message
     */
    virtual void processPacket(const velodyne_msgs::VelodynePacket *pkt,
                               const std::string &frame_id) = 0;
#endif // DEPRECATED_RAWDATA     // define DEPRECATED methods & types

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

  protected:

    /** configuration parameters */
    std::string anglesFile_;            ///< correction angles file name

    /** correction angles indexed by laser within bank
     *
     * \todo combine them into a single array, lower followed by upper
     */
    correction_angles lower_[SCANS_PER_BLOCK];
    correction_angles upper_[SCANS_PER_BLOCK];

#ifdef DEPRECATED_RAWDATA         // define DEPRECATED methods & types
    /** latest raw scan message received */
    velodyne_msgs::VelodyneScan::ConstPtr rawScan_;
#endif // DEPRECATED_RAWDATA     // define DEPRECATED methods & types
  };

  ////////////////////////////////////////////////////////////////////
  // RawDataScans derived class
  ////////////////////////////////////////////////////////////////////

#ifdef DEPRECATED_RAWDATA         // define DEPRECATED methods & types
  /** \brief A single laser scan in Velodyne's frame of reference.
   *
   *   pitch is relative to the plane of the unit (in its frame of
   *   reference): positive is above, negative is below
   *
   *   heading is relative to the front of the unit (the outlet is the back):
   *   positive is clockwise  (yaw)
   */
  typedef struct laserscan
  {
    float range;                        ///< in meters
    float heading;                      ///< in radians
    float pitch;                        ///< in radians
    uint16_t revolution;
    uint8_t  laser_number;
    uint8_t  intensity;
  } laserscan_t;

  /** \brief type of callback function to receive laser scans
   *
   *  \param scan vector containing the laser scans for a single packet
   *  \param time ROS time stamp of packet
   *  \param frame_id ROS frame ID of packet
   */
  typedef boost::function<void(const std::vector<laserscan_t> &scan,
                               ros::Time time,
                               const std::string &frame_id)> scanCallback;

#endif // DEPRECATED_RAWDATA     // define DEPRECATED methods & types

  /** \brief Convert Velodyne raw input to Polar format */
  class RawDataScans: public RawData
  {
  public:

    RawDataScans();

#ifdef DEPRECATED_RAWDATA         // define DEPRECATED methods & types
    virtual void processPacket(const velodyne_msgs::VelodynePacket *pkt,
                               const std::string &frame_id);

    /** \brief Subscribe to laser scans.
     *
     * \param node the ros::NodeHandle to use to subscribe.
     * \param base_topic the topic to subscribe to.
     * \param queue_size the subscription queue size
     * \param callback function or method to receive scan data
     * \param transport_hints optional transport hints for this subscription
     */
    ros::Subscriber subscribe(ros::NodeHandle node,
                              const std::string &topic,
                              uint32_t queue_size,
                              const scanCallback callback,
                              const ros::TransportHints &transport_hints
                                      = ros::TransportHints())
    {
      ROS_INFO("generic scan callback defined");
      cb_ = callback;
      return node.subscribe(topic, queue_size,
                            &RawData::processScan, (RawData *) this,
                            transport_hints);
    }

  private:
    scanCallback cb_;                   ///< scan data callback

  protected:
    // derived class data
    std::vector<laserscan_t> scans_;

    void packet2scans(const raw_packet_t *raw, laserscan_t *scans);

#else // not DEPRECATED_RAWDATA     // define new methods & types

  protected:
    void packet2scans(const raw_packet_t &raw, VPolar &scans);

#endif // DEPRECATED_RAWDATA     // define DEPRECATED methods & types
  };


  ////////////////////////////////////////////////////////////////////
  // RawDataXYZ derived class
  ////////////////////////////////////////////////////////////////////

#ifdef DEPRECATED_RAWDATA         // define DEPRECATED methods & types
  /** \brief a single laser scan in XYZ format
   *
   *  Data are in the Velodyne's frame of reference.
   */
  typedef struct laserscan_xyz
  {
    float x, y, z;
    float heading;
    uint16_t revolution;
    uint8_t  laser_number;
    uint8_t  intensity;
  } laserscan_xyz_t;

  /** \brief vector of XYZ laser scans */
  typedef std::vector<laserscan_xyz_t> xyz_scans_t;

  /** \brief type of callback function to receive XYZ laser scans
   *
   *  \param scan vector containing the XYZ scans for a single packet
   *  \param stamp ROS time stamp of packet
   *  \param frame_id ROS frame ID of packet
   */
  typedef boost::function<void(const xyz_scans_t &scan,
                               ros::Time stamp,
                               const std::string &frame_id)> xyzCallback;
#endif // DEPRECATED_RAWDATA     // define DEPRECATED methods & types

  /** \brief Convert Velodyne raw input to XYZ format */
  class RawDataXYZ: public RawDataScans
  {
  public:

    RawDataXYZ();

#ifdef DEPRECATED_RAWDATA         // define DEPRECATED methods & types
    virtual void processPacket(const velodyne_msgs::VelodynePacket *pkt,
                               const std::string &frame_id);

    /** \brief Subscribe to XYZ laser scans.
     *
     * \param node the ros::NodeHandle to use to subscribe.
     * \param base_topic the topic to subscribe to.
     * \param queue_size the subscription queue size
     * \param callback function or method to receive XYZ data
     * \param transport_hints optional transport hints for this subscription
     */
    ros::Subscriber subscribe(ros::NodeHandle node,
                              const std::string &topic,
                              uint32_t queue_size,
                              const xyzCallback callback,
                              const ros::TransportHints &transport_hints
                                      = ros::TransportHints())
    {
      ROS_INFO("XYZ callback subscribed");
      cb_ = callback;
      return node.subscribe(topic, queue_size,
                            &RawData::processScan, (RawData *) this,
                            transport_hints);
    }

  protected:
    xyz_scans_t xyzScans_;              ///< vector of XYZ scans

    void scan2xyz(const laserscan_t *scan, laserscan_xyz_t *point);

  private:
    xyzCallback cb_;                    ///< XYZ packet callback

#else  // not DEPRECATED_RAWDATA     // define new methods & types

  public:
    void unpack(const velodyne_msgs::VelodynePacket &pkt, VPointCloud::Ptr &pc);

#endif // DEPRECATED_RAWDATA     // define DEPRECATED methods & types
  };

} // namespace velodyne_pointcloud

#endif // __VELODYNE_RAWDATA_H
