// Copyright 2007, 2009, 2010, 2012, 2019 Yaxin Liu, Patrick Beeson, Jack O'Quin, Joshua Whitley  // NOLINT
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef VELODYNE_POINTCLOUD__RAWDATA_HPP_
#define VELODYNE_POINTCLOUD__RAWDATA_HPP_

#include <pcl/point_cloud.h>

#include <rclcpp/rclcpp.hpp>
#include <velodyne_msgs/msg/velodyne_packet.hpp>

#include <memory>
#include <string>

#include "velodyne_pointcloud/calibration.hpp"
#include "velodyne_pointcloud/datacontainerbase.hpp"

namespace velodyne_rawdata
{
/**
 * Raw Velodyne packet constants and structures.
 */
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const float ROTATION_RESOLUTION = 0.01f;     // [deg]
static const uint16_t ROTATION_MAX_UNITS = 36000u;  // [deg/100]

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static const int VLP16_FIRINGS_PER_BLOCK = 2;
static const int VLP16_SCANS_PER_FIRING = 16;
static const float VLP16_BLOCK_TDURATION = 110.592f;  // [µs]
static const float VLP16_DSR_TOFFSET = 2.304f;        // [µs]
static const float VLP16_FIRING_TOFFSET = 55.296f;    // [µs]

/** \brief Raw Velodyne data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
struct raw_block
{
  uint16_t header;    ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;  ///< 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];
};

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes {
  uint16_t uint;
  uint8_t bytes[2];
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
struct raw_packet
{
  raw_block blocks[BLOCKS_PER_PACKET];
  uint16_t revolution;
  uint8_t status[PACKET_STATUS_SIZE];
};

/** \brief Velodyne data conversion class */
class RawData final
{
public:
  explicit RawData(const std::string & calibration_file);

  void unpack(const velodyne_msgs::msg::VelodynePacket & pkt, DataContainerBase & data);

  void setParameters(double min_range, double max_range, double view_direction, double view_width);

  int scansPerPacket() const;

  int numLasers() const;

private:
  /** configuration parameters */
  struct Config
  {
    double min_range;             ///< minimum range to publish
    double max_range;             ///< maximum range to publish
    int min_angle;                ///< minimum angle to publish
    int max_angle;                ///< maximum angle to publish
  };
  Config config_{};

  /**
   * Calibration file
   */
  std::unique_ptr<velodyne_pointcloud::Calibration> calibration_;
  float sin_rot_table_[ROTATION_MAX_UNITS]{};
  float cos_rot_table_[ROTATION_MAX_UNITS]{};

  /** add private function to handle the VLP16 **/
  void unpack_vlp16(const velodyne_msgs::msg::VelodynePacket & pkt, DataContainerBase & data);
};

}  // namespace velodyne_rawdata

#endif  // VELODYNE_POINTCLOUD__RAWDATA_HPP_
