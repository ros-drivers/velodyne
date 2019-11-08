// Copyright 2007, 2009, 2010, 2012, 2019 Austin Robot Technology, Patrick Beeson, Jack O'Quin, Joshua Whitley  // NOLINT
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

#include <angles/angles.h>

#include <velodyne_msgs/msg/velodyne_packet.hpp>

#include <cmath>
#include <memory>
#include <string>

#include "velodyne_pointcloud/datacontainerbase.hpp"
#include "velodyne_pointcloud/rawdata.hpp"

namespace velodyne_rawdata
{

inline float square(float val)
{
  return val * val;
}

////////////////////////////////////////////////////////////////////////
//
// RawData base class implementation
//
////////////////////////////////////////////////////////////////////////

RawData::RawData(const std::string & calibration_file)
{
  calibration_ = std::make_unique<velodyne_pointcloud::Calibration>(calibration_file);

  // RCLCPP_INFO(this->get_logger(), "Number of lasers: %d.",  calibration_->num_lasers);

  // Set up cached values for sin and cos of all the possible headings
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
    cos_rot_table_[rot_index] = ::cosf(rotation);
    sin_rot_table_[rot_index] = ::sinf(rotation);
  }
}

/** Update parameters: conversions and update */
void RawData::setParameters(
  double min_range, double max_range, double view_direction, double view_width)
{
  config_.min_range = min_range;
  config_.max_range = max_range;

  // converting angle parameters into the velodyne reference (rad)
  double tmp_min_angle = view_direction + view_width / 2;
  double tmp_max_angle = view_direction - view_width / 2;

  // computing positive modulo to keep theses angles into [0;2*M_PI]
  tmp_min_angle = ::fmod(::fmod(tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  tmp_max_angle = ::fmod(::fmod(tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

  // converting into the hardware velodyne ref (negative yaml and degrees)
  // adding 0.5 performs a centered double to int conversion
  config_.min_angle = std::lround(100.0 * (2.0 * M_PI - tmp_min_angle) * 180.0 / M_PI);
  config_.max_angle = std::lround(100.0 * (2.0 * M_PI - tmp_max_angle) * 180.0 / M_PI);

  if (config_.min_angle == config_.max_angle) {
    // avoid returning empty cloud if min_angle = max_angle
    config_.min_angle = 0;
    config_.max_angle = 36000;
  }
}

int RawData::scansPerPacket() const
{
  if (calibration_->num_lasers == 16) {
    return BLOCKS_PER_PACKET * VLP16_FIRINGS_PER_BLOCK *
           VLP16_SCANS_PER_FIRING;
  }
  return BLOCKS_PER_PACKET * SCANS_PER_BLOCK;
}

int RawData::numLasers() const
{
  return calibration_->num_lasers;
}

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack(const velodyne_msgs::msg::VelodynePacket & pkt, DataContainerBase & data)
{
  // RCLCPP_DEBUG(this->get_logger(), "Received packet, time: %s", pkt.stamp);

  /** special parsing for the VLP16 **/
  if (calibration_->num_lasers == 16) {
    unpack_vlp16(pkt, data);
    return;
  }

  const raw_packet * raw = reinterpret_cast<const raw_packet *>(&pkt.data[0]);

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    // upper bank lasers are numbered [0..31]
    // NOTE: this is a change from the old velodyne_common implementation

    int bank_origin = 0;

    if (raw->blocks[i].header == LOWER_BANK) {
      // lower bank lasers are [32..63]
      bank_origin = 32;
    }

    for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
      float x, y, z;
      float intensity;
      const uint8_t laser_number = j + bank_origin;

      const velodyne_pointcloud::LaserCorrection & corrections =
        calibration_->laser_corrections[laser_number];

      /** Position Calculation */
      const raw_block & block = raw->blocks[i];
      union two_bytes tmp{};
      tmp.bytes[0] = block.data[k];
      tmp.bytes[1] = block.data[k + 1];

      if (tmp.bytes[0] == 0 && tmp.bytes[1] == 0) {  // no laser beam return
        continue;
      }

      /*condition added to avoid calculating points which are not
        in the interesting defined area (min_angle < area < max_angle)*/
      if ((config_.min_angle < config_.max_angle &&
        block.rotation >= config_.min_angle &&
        block.rotation <= config_.max_angle) ||
        (config_.min_angle > config_.max_angle &&
        (block.rotation <= config_.max_angle ||
        block.rotation >= config_.min_angle)))
      {
        float distance = tmp.uint * calibration_->distance_resolution_m;
        distance += corrections.dist_correction;

        float cos_vert_angle = corrections.cos_vert_correction;
        float sin_vert_angle = corrections.sin_vert_correction;
        float cos_rot_correction = corrections.cos_rot_correction;
        float sin_rot_correction = corrections.sin_rot_correction;

        // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
        // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
        float cos_rot_angle =
          cos_rot_table_[block.rotation] * cos_rot_correction +
          sin_rot_table_[block.rotation] * sin_rot_correction;
        float sin_rot_angle =
          sin_rot_table_[block.rotation] * cos_rot_correction -
          cos_rot_table_[block.rotation] * sin_rot_correction;

        float horiz_offset = corrections.horiz_offset_correction;
        float vert_offset = corrections.vert_offset_correction;

        // Compute the distance in the xy plane (w/o accounting for rotation)
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

        // Calculate temporal X, use absolute value.
        float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
        // Calculate temporal Y, use absolute value
        float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

        if (xx < 0.0f) {
          xx = -xx;
        }

        if (yy < 0.0f) {
          yy = -yy;
        }

        // Get 2points calibration values,Linear interpolation to get distance
        // correction for X and Y, that means distance correction use
        // different value at different distance
        float distance_corr_x = 0;
        float distance_corr_y = 0;

        if (corrections.two_pt_correction_available) {
          distance_corr_x =
            (corrections.dist_correction - corrections.dist_correction_x) *
            (xx - 2.4f) / (25.04f - 2.4f) +
            corrections.dist_correction_x;
          distance_corr_x -= corrections.dist_correction;
          distance_corr_y =
            (corrections.dist_correction - corrections.dist_correction_y) *
            (yy - 1.93f) / (25.04f - 1.93f) +
            corrections.dist_correction_y;
          distance_corr_y -= corrections.dist_correction;
        }

        float distance_x = distance + distance_corr_x;
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
        // the expression wiht '-' is proved to be better than the one with '+'
        x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

        float distance_y = distance + distance_corr_y;
        xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

        // Using distance_y is not symmetric, but the velodyne manual
        // does this.
        /**the new term of 'vert_offset * cos_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

        /** Use standard ROS coordinate system (right-hand rule) */
        float x_coord = y;
        float y_coord = -x;
        float z_coord = z;

        /** Intensity Calculation */

        float min_intensity = corrections.min_intensity;
        float max_intensity = corrections.max_intensity;

        intensity = raw->blocks[i].data[k + 2];

        float focal_offset = 256.0f *
          (1.0f - corrections.focal_distance / 13100.0f) *
          (1.0f - corrections.focal_distance / 13100.0f);
        float focal_slope = corrections.focal_slope;
        intensity += focal_slope *
          (std::abs(focal_offset - 256.0f * square((1.0f - distance) / 65535.0f)));
        intensity = (intensity < min_intensity) ? min_intensity : intensity;
        intensity = (intensity > max_intensity) ? max_intensity : intensity;

        data.addPoint(
          x_coord, y_coord, z_coord, corrections.laser_ring,
          distance, intensity);
      }
    }

    data.newLine();
  }
}

/** @brief convert raw VLP16 packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack_vlp16(const velodyne_msgs::msg::VelodynePacket & pkt, DataContainerBase & data)
{
  float azimuth;
  float azimuth_diff;
  int raw_azimuth_diff;
  float last_azimuth_diff = 0.0f;
  float azimuth_corrected_f;
  int azimuth_corrected;
  float x, y, z;
  float intensity;

  const raw_packet * raw = reinterpret_cast<const raw_packet *>(&pkt.data[0]);

  for (int block = 0; block < BLOCKS_PER_PACKET; block++) {
    // ignore packets with mangled or otherwise different contents
    if (UPPER_BANK != raw->blocks[block].header) {
      return;  // bad packet: skip the rest
    }

    // Calculate difference between current and next block's azimuth angle.
    azimuth = static_cast<float>(raw->blocks[block].rotation);

    if (block < (BLOCKS_PER_PACKET - 1)) {
      raw_azimuth_diff = raw->blocks[block + 1].rotation - raw->blocks[block].rotation;
      azimuth_diff = static_cast<float>((36000 + raw_azimuth_diff) % 36000);

      // some packets contain an angle overflow where azimuth_diff < 0
      if (raw_azimuth_diff < 0) {
        // raw->blocks[block+1].rotation - raw->blocks[block].rotation < 0)
        // RCLCPP_WARN(
        //   get_logger(), "Packet containing angle overflow, first angle: %u second angle: %u",
        //   raw->blocks[block].rotation, raw->blocks[block+1].rotation);
        // if last_azimuth_diff was not zero, we can assume that the velodyne's
        // speed did not change very much and use the same difference
        if (last_azimuth_diff > 0) {
          azimuth_diff = last_azimuth_diff;
        } else {
          // otherwise we are not able to use this data
          // TODO(somebody): we might just not use the second 16 firings
          continue;
        }
      }

      last_azimuth_diff = azimuth_diff;
    } else {
      azimuth_diff = last_azimuth_diff;
    }

    for (int firing = 0, k = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++) {
      for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE) {
        velodyne_pointcloud::LaserCorrection & corrections = calibration_->laser_corrections[dsr];

        /** Position Calculation */
        union two_bytes tmp{};
        tmp.bytes[0] = raw->blocks[block].data[k];
        tmp.bytes[1] = raw->blocks[block].data[k + 1];

        /** correct for the laser rotation as a function of timing during the firings **/
        azimuth_corrected_f =
          azimuth + (azimuth_diff * ((dsr * VLP16_DSR_TOFFSET) +
          (firing * VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
        azimuth_corrected = (static_cast<int>(std::round(azimuth_corrected_f)) % 36000);

        /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
        if ((azimuth_corrected >= config_.min_angle &&
          azimuth_corrected <= config_.max_angle &&
          config_.min_angle < config_.max_angle) ||
          (config_.min_angle > config_.max_angle &&
          (azimuth_corrected <= config_.max_angle ||
          azimuth_corrected >= config_.min_angle)))
        {
          // convert polar coordinates to Euclidean XYZ
          float distance = tmp.uint * calibration_->distance_resolution_m;
          distance += corrections.dist_correction;

          float cos_vert_angle = corrections.cos_vert_correction;
          float sin_vert_angle = corrections.sin_vert_correction;
          float cos_rot_correction = corrections.cos_rot_correction;
          float sin_rot_correction = corrections.sin_rot_correction;

          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
          float cos_rot_angle =
            cos_rot_table_[azimuth_corrected] * cos_rot_correction +
            sin_rot_table_[azimuth_corrected] * sin_rot_correction;
          float sin_rot_angle =
            sin_rot_table_[azimuth_corrected] * cos_rot_correction -
            cos_rot_table_[azimuth_corrected] * sin_rot_correction;

          float horiz_offset = corrections.horiz_offset_correction;
          float vert_offset = corrections.vert_offset_correction;

          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

          if (xx < 0.0f) {
            xx = -xx;
          }

          if (yy < 0.0f) {
            yy = -yy;
          }

          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;

          if (corrections.two_pt_correction_available) {
            distance_corr_x =
              (corrections.dist_correction - corrections.dist_correction_x) *
              (xx - 2.4f) / (25.04f - 2.4f) + corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y =
              (corrections.dist_correction - corrections.dist_correction_y) *
              (yy - 1.93f) / (25.04f - 1.93f) + corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }

          float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

          float distance_y = distance + distance_corr_y;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

          // Using distance_y is not symmetric, but the velodyne manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

          /** Use standard ROS coordinate system (right-hand rule) */
          float x_coord = y;
          float y_coord = -x;
          float z_coord = z;

          /** Intensity Calculation */
          float min_intensity = corrections.min_intensity;
          float max_intensity = corrections.max_intensity;

          intensity = raw->blocks[block].data[k + 2];

          float focal_offset = 256.0f * square(1.0f - corrections.focal_distance / 13100.0f);
          float focal_slope = corrections.focal_slope;
          intensity += focal_slope *
            (std::abs(focal_offset - 256.0f * square((1.0f - distance) / 65535.0f)));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;

          data.addPoint(
            x_coord, y_coord, z_coord, corrections.laser_ring,
            distance, intensity);
        }
      }

      data.newLine();
    }
  }
}

}  // namespace velodyne_rawdata
