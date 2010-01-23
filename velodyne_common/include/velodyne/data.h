/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Velodyne HDL-64E 3D LIDAR data accessors
 *
 *  \ingroup velodyne
 *
 *  These classes unpack raw Velodyne LIDAR packets into several
 *  useful formats.
 *
 *     velodyne::Data -- virtual base class for unpacking data into
 *                      various formats
 *
 *     velodyne::DataScans -- derived class, unpacks into vector of
 *                      individual laser scans
 *
 *     velodyne::DataXYZ -- derived class, unpacks into XYZ format
 *
 *  \todo make a separate header for each class?
 *
 *  \author Yaxin Liu
 *  \author Patrick Beeson
 *  \author Jack O'Quin
 */


#ifndef __VELODYNE_DATA_H
#define __VELODYNE_DATA_H

#include <errno.h>
#include <stdint.h>
#include <string>

#include <velodyne_common/RawScan.h>

namespace velodyne
{

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
    int   enabled;
  };

  /** \brief type of callback function to receive raw data for one revolution.
   *
   * \param raw -> buffer containing raw packet contents
   * \param npackets number of packets in the buffer
   *
   * Use getMsgHeader() to access the ROS message header containing
   * time stamp, sequence number, and frame ID.
   */
  typedef void (*raw_callback_t)(const raw_packet_t *raw, size_t npackets);

  /** \brief Base Velodyne data class -- not used directly. */
  class Data
  {
  public:
    Data(std::string ofile="",
         std::string anglesFile="")
      {
        ofile_ = ofile;
        anglesFile_ = anglesFile;

        ofp_ = NULL;
        rawCB_ = NULL;
        memset(&upper_, 0, sizeof(upper_));
        memset(&lower_, 0, sizeof(lower_));
      }

    virtual ~Data() {}

    /** \brief Get latest ROS message header.
     *
     * \returns NULL, if no message available;
     *          pointer to latest RawScan message header, otherwise.
     *
     *  This will always reflect the current message while any
     *  callback function is running.
     */
    virtual const roslib::Header *getMsgHeader(void) const
    {
      if (rawScan_)
        return &rawScan_->header;
      else
        return NULL;
    }

    /** \brief Get ROS parameters.
     *
     *  ROS parameter settings override constructor options
     *
     * \returns 0, if successful;
     *          errno value, for failure
     */
    virtual int getParams(void);

    /** \brief Print data to human-readable file.
     *
     * \returns 0, if successful;
     *          errno value, for failure
     */
    virtual int print(void)
    {
      if (uninitialized_)
        return EBADF;                   // bad file descriptor
      return 0;
    }

    /** \brief handle raw scan ROS topic message
      
        This is the main entry point for handling ROS messages from
        the /velodyne/rawscan topic, published by the velodyne_read
        (device driver) node.  This is \b not a virtual function;
        derived classes may replace processRaw(), instead.
      
        Typically, velodyne_read publishes all the data for a single
        rotation of the device in each message.
      
        Subscribe via the method-oriented ROS interface, like this:
      
        \verbatim
           velodyne::Data *data = new velodyne::Data();
     
           ros::Subscriber velodyne_scan =
             node.subscribe("velodyne/rawscan", 1,
                            &velodyne::Data::processRawScan, data,
                            ros::TransportHints().tcpNoDelay(true));
        \endverbatim
     */
    void processRawScan(const velodyne_common::RawScan::ConstPtr &raw_scan);

    /** \brief Process raw Velodyne packets for an entire revolution.
     *
     * \param raw -> buffer containing raw packet contents
     * \param npackets number of packets in the buffer
     */
    virtual void processRaw(const raw_packet_t *raw, size_t npackets);

    /** \brief Set up for data processing.
     *
     *  Perform initializations needed before data processing can
     *  begin:
     *
     *    - open output file (if any)
     *    - read device-specific correction angles
     *
     *  \returns 0 if successful;
     *           errno value for failure
     */
    virtual int setup(void);

    /** \brief Shut down data processing. */
    virtual void shutdown(void)
    {
      ros::shutdown();                  // break out of main ros::spin()
      uninitialized_ = true;
    }

    /** \brief Subscribe to raw packets for a each revolution. */
    virtual void subscribeRaw(raw_callback_t rawCB)
    {
      ROS_INFO("raw callback defined");
      rawCB_ = rawCB;
    }

  protected:

    /** configuration parameters */
    std::string ofile_;                 ///< output file name for print()
    std::string anglesFile_;            ///< correction angles file name

    /** runtime state */
    FILE *ofp_;                         ///< output file descriptor
    raw_callback_t rawCB_;              ///< raw data callback
    bool uninitialized_;                ///< false after successful setup()

    /** latest raw scan message received */
    velodyne_common::RawScan::ConstPtr rawScan_;

    /** correction angles indexed by laser within bank
     *
     * \todo combine them into a single array, lower followed by upper
     */
    correction_angles lower_[SCANS_PER_BLOCK];
    correction_angles upper_[SCANS_PER_BLOCK];
  };


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
   * \param scan -> vector containing the laser scans for a complete
   *                    revolution in Velodyne frame of reference.
   *
   * Use getMsgHeader() to access the ROS message header containing
   * time stamp, sequence number, and frame ID.
   */
  typedef void (*scans_callback_t)(const std::vector<laserscan_t> &scan);

  /** \brief Convert Velodyne raw input to laserscans format */
  class DataScans: public Data
  {
  public:
    DataScans(std::string ofile="",
              std::string anglesFile=""):
      Data(ofile, anglesFile)
    {
      // reserve vector space before processing, we don't want to
      // reallocate in real time
      scans_.resize(SCANS_PER_REV);
      scansCB_ = NULL;
    }

    virtual int print(void);
    virtual void processRaw(const raw_packet_t *raw, size_t npackets);

    /** \brief Subscribe to laser scans for each revolution. */
    virtual void subscribeScans(scans_callback_t scansCB)
    {
      ROS_INFO("scans callback defined");
      scansCB_ = scansCB;
    }

  protected:
    void packet2scans(const raw_packet_t *raw, laserscan_t *scans);

    // derived class data
    scans_callback_t scansCB_;
    std::vector<laserscan_t> scans_;
  };


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

  /** \brief type of callback function to receive XYZ laser scans
   *
   * \param scan -> vector containing the laser scans for a complete
   *                    revolution in XYZ format using Velodyne frame
   *                    of reference.
   *
   * Use getMsgHeader() to access the ROS message header containing
   * time stamp, sequence number, and frame ID.
   */
  typedef void (*xyz_callback_t)(const std::vector<laserscan_xyz_t> &scan);

  /** \brief Convert Velodyne raw input to XYZ format */
  class DataXYZ: public DataScans
  {
  public:
    DataXYZ(std::string ofile="",
            std::string anglesFile=""):
      DataScans(ofile, anglesFile)
    {
      // reserve vector space before processing, we don't want to
      // reallocate in real time
      xyzScans_.resize(SCANS_PER_REV);
      xyzCB_ = NULL;
    }

    virtual int print(void);
    virtual void processRaw(const raw_packet_t *raw, size_t npackets);

    /** \brief Subscribe to XYZ laser scans for each revolution. */
    virtual void subscribeXYZ(xyz_callback_t xyzCB)
    {
      ROS_INFO("XYZ callback defined");
      xyzCB_ = xyzCB;
    }

  protected:
    void scan2xyz(const laserscan_t *scan, laserscan_xyz_t *point);

    // derived class data
    xyz_callback_t xyzCB_;
    std::vector<laserscan_xyz_t> xyzScans_;
  };


} // velodyne namespace

#endif // __VELODYNE_DATA_H
