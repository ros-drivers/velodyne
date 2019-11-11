// Copyright 2012, 2019 Austin Robot Technology, Jack O'Quin, Joshua Whitley, Sebastian Pütz  // NOLINT
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

#ifndef VELODYNE_POINTCLOUD__DATACONTAINERBASE_HPP_
#define VELODYNE_POINTCLOUD__DATACONTAINERBASE_HPP_

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>

namespace velodyne_rawdata
{
class DataContainerBase
{
public:
  explicit DataContainerBase(
    const double min_range, const double max_range, const std::string & target_frame,
    const std::string & fixed_frame, const unsigned int init_width, const unsigned int init_height,
    const bool is_dense, const unsigned int scans_per_packet,
    tf2::BufferCore & buffer, int fields, ...)
  : config_(min_range, max_range, target_frame, fixed_frame,
      init_width, init_height, is_dense, scans_per_packet),
    tf_buffer_(buffer)
  {
    va_list vl;
    cloud.fields.clear();
    cloud.fields.reserve(fields);
    va_start(vl, fields);
    int offset = 0;

    for (int i = 0; i < fields; ++i) {
      // Create the corresponding PointField
      std::string name(va_arg(vl, char *));
      int count(va_arg(vl, int));
      int datatype(va_arg(vl, int));
      offset = addPointField(cloud, name, count, datatype, offset);
    }

    va_end(vl);
    cloud.point_step = offset;
    cloud.width = config_.init_width;
    cloud.height = config_.init_height;
    cloud.is_dense = static_cast<uint8_t>(config_.is_dense);
    cloud.row_step = cloud.width * cloud.point_step;
  }

  virtual ~DataContainerBase() {}

  struct Config final
  {
    double min_range;          ///< minimum range to publish
    double max_range;          ///< maximum range to publish
    std::string target_frame;  ///< target frame to transform a point
    std::string fixed_frame;   ///< fixed frame used for transform
    unsigned int init_width;
    unsigned int init_height;
    bool is_dense;
    unsigned int scans_per_packet;
    bool transform;  ///< enable / disable transform points

    explicit Config(
      double min_range, double max_range, const std::string & target_frame,
      const std::string & fixed_frame, unsigned int init_width,
      unsigned int init_height, bool is_dense, unsigned int scans_per_packet)
    : min_range(min_range),
      max_range(max_range),
      target_frame(target_frame),
      fixed_frame(fixed_frame),
      init_width(init_width),
      init_height(init_height),
      is_dense(is_dense),
      scans_per_packet(scans_per_packet),
      transform(fixed_frame != target_frame)
    {
    }
  };

  virtual void setup(const velodyne_msgs::msg::VelodyneScan::SharedPtr scan_msg)
  {
    cloud.header = scan_msg->header;
    cloud.width = config_.init_width;
    cloud.height = config_.init_height;
    cloud.is_dense = static_cast<uint8_t>(config_.is_dense);
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.data.resize(scan_msg->packets.size() * config_.scans_per_packet * cloud.point_step);
    // Clear out the last data; this is important in the organized cloud case
    std::fill(cloud.data.begin(), cloud.data.end(), 0);
    if (config_.transform) {
      computeTransformation(scan_msg->header.stamp);
    }
  }

  virtual void addPoint(
    float x, float y, float z, const uint16_t ring, const float distance,
    const float intensity) = 0;
  virtual void newLine() = 0;

  const sensor_msgs::msg::PointCloud2 & finishCloud()
  {
    cloud.data.resize(cloud.point_step * cloud.width * cloud.height);
    // If config_.target_frame is empty (the default), we use the frame_id that
    // came in during the initial VelodyneScan (set by setup()).  If it is
    // set to something, then we override that value.
    if (!config_.target_frame.empty()) {
      cloud.header.frame_id = config_.target_frame;
    }
    return cloud;
  }

  void configure(
    const double min_range, const double max_range, const std::string & fixed_frame,
    const std::string & target_frame)
  {
    config_.min_range = min_range;
    config_.max_range = max_range;
    config_.fixed_frame = fixed_frame;
    config_.target_frame = target_frame;

    config_.transform = fixed_frame != target_frame;
  }

protected:
  sensor_msgs::msg::PointCloud2 cloud;

  inline void vectorTfToEigen(tf2::Vector3 & tf_vec, Eigen::Vector3f & eigen_vec)
  {
    eigen_vec(0) = tf_vec[0];
    eigen_vec(1) = tf_vec[1];
    eigen_vec(2) = tf_vec[2];
  }

  void computeTransformation(const rclcpp::Time & time)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      const std::chrono::nanoseconds dur(time.nanoseconds());
      std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> time(dur);
      transform = tf_buffer_.lookupTransform(config_.target_frame, cloud.header.frame_id, time);
    } catch (tf2::LookupException & e) {
      return;
    } catch (tf2::ExtrapolationException & e) {
      return;
    }

    tf2::Quaternion quaternion(
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w);
    Eigen::Quaternionf rotation(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());

    Eigen::Vector3f eigen_origin;
    tf2::Vector3 origin(
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);
    vectorTfToEigen(origin, eigen_origin);
    Eigen::Translation3f translation(eigen_origin);
    transformation = translation * rotation;
  }

  inline void transformPoint(float & x, float & y, float & z)
  {
    Eigen::Vector3f p = transformation * Eigen::Vector3f(x, y, z);
    x = p.x();
    y = p.y();
    z = p.z();
  }

  inline bool pointInRange(float range)
  {
    return range >= config_.min_range && range <= config_.max_range;
  }

  Config config_;
  tf2::BufferCore & tf_buffer_;
  Eigen::Affine3f transformation;
};
}  // namespace velodyne_rawdata

#endif  // VELODYNE_POINTCLOUD__DATACONTAINERBASE_HPP_
