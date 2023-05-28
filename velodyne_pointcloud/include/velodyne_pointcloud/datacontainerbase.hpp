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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <velodyne_msgs/msg/velodyne_scan.hpp>

namespace velodyne_rawdata
{
class DataContainerBase
{
public:
  explicit DataContainerBase(
    const double min_range, const double max_range, const std::string & target_frame,
    const std::string & fixed_frame, const unsigned int init_width, const unsigned int init_height,
    const bool is_dense, const unsigned int scans_per_packet, rclcpp::Clock::SharedPtr clock,
    int fields, ...)
  : config_(min_range, max_range, target_frame, fixed_frame,
      init_width, init_height, is_dense, scans_per_packet),
    clock_(clock)
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
    std::string target_frame;  ///< output frame of final point cloud
    std::string fixed_frame;   ///< world fixed frame for ego motion compenstation
    unsigned int init_width;
    unsigned int init_height;
    bool is_dense;
    unsigned int scans_per_packet;

    Config(
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
      scans_per_packet(scans_per_packet)
    {
    }
  };

  virtual void setup(const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr scan_msg)
  {
    sensor_frame_ = scan_msg->header.frame_id;
    manage_tf_buffer();

    cloud.header.stamp = scan_msg->header.stamp;
    cloud.width = config_.init_width;
    cloud.height = config_.init_height;
    cloud.is_dense = static_cast<uint8_t>(config_.is_dense);
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.data.resize(scan_msg->packets.size() * config_.scans_per_packet * cloud.point_step);
    // Clear out the last data; this is important in the organized cloud case
    std::fill(cloud.data.begin(), cloud.data.end(), 0);
  }

  virtual void addPoint(
    float x, float y, float z, const uint16_t ring, const float distance,
    const float intensity, const float time) = 0;
  virtual void newLine() = 0;

  const sensor_msgs::msg::PointCloud2 & finishCloud()
  {
    cloud.data.resize(cloud.point_step * cloud.width * cloud.height);
    cloud.row_step = cloud.point_step * cloud.width;

    // If config_.target_frame is empty (the default), we use the frame_id that
    // came in during the initial VelodyneScan (set by setup()).  If it is
    // set to something, then we override that value.
    if (!config_.target_frame.empty()) {
      cloud.header.frame_id = config_.target_frame;
    } else if (!config_.fixed_frame.empty()) {
      cloud.header.frame_id = config_.fixed_frame;
    } else {
      cloud.header.frame_id = sensor_frame_;
    }

    return cloud;
  }

  void manage_tf_buffer()
  {
    // check if sensor frame is already known, if not don't prepare tf buffer until setup was called
    if (sensor_frame_.empty()) {
      return;
    }

    // avoid doing transformation when sensor_frame equals target frame
    // and no ego motion compensation is perfomed
    if (config_.fixed_frame.empty() && sensor_frame_ == config_.target_frame) {
      // when the string is empty the points will not be transformed later on
      config_.target_frame = "";
      return;
    }

    // only use somewhat resource intensive tf listener when transformations are necessary
    if (!config_.fixed_frame.empty() || !config_.target_frame.empty()) {
      if (!tf_buffer_) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      }
    } else {
      tf_listener_.reset();
      tf_buffer_.reset();
    }
  }

  void configure(
    const double min_range, const double max_range, const std::string & fixed_frame,
    const std::string & target_frame)
  {
    config_.min_range = min_range;
    config_.max_range = max_range;
    config_.fixed_frame = fixed_frame;
    config_.target_frame = target_frame;

    manage_tf_buffer();
  }

  inline bool calculateTransformMatrix(
    Eigen::Affine3f & matrix, const std::string & target_frame,
    const std::string & source_frame, const rclcpp::Time & time)
  {
    if (!tf_buffer_) {
      RCLCPP_ERROR(rclcpp::get_logger("velodyne_pointcloud"), "tf buffer was not initialized yet");
      return false;
    }

    geometry_msgs::msg::TransformStamped msg;
    try {
      msg = tf_buffer_->lookupTransform(
        target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.2));
    } catch (tf2::LookupException & e) {
      RCLCPP_ERROR(rclcpp::get_logger("velodyne_pointcloud"), "%s", e.what());
      return false;
    } catch (tf2::ExtrapolationException & e) {
      RCLCPP_ERROR(rclcpp::get_logger("velodyne_pointcloud"), "%s", e.what());
      return false;
    }

    const auto & quaternion = msg.transform.rotation;
    Eigen::Quaternionf rotation(quaternion.w, quaternion.x, quaternion.y, quaternion.z);

    const auto & origin = msg.transform.translation;
    Eigen::Translation3f translation(origin.x, origin.y, origin.z);

    matrix = translation * rotation;
    return true;
  }

  inline bool computeTransformToTarget(const rclcpp::Time & scan_time)
  {
    if (config_.target_frame.empty()) {
      // no need to calculate transform -> success
      return true;
    }
    std::string & source_frame = config_.fixed_frame.empty() ? sensor_frame_ : config_.fixed_frame;
    return calculateTransformMatrix(
      tf_matrix_to_target_, config_.target_frame, source_frame, scan_time);
  }

  inline bool computeTransformToFixed(const rclcpp::Time & packet_time)
  {
    if (config_.fixed_frame.empty()) {
      // no need to calculate transform -> success
      return true;
    }
    std::string & source_frame = sensor_frame_;
    return calculateTransformMatrix(
      tf_matrix_to_fixed_, config_.fixed_frame, source_frame, packet_time);
  }

protected:
  sensor_msgs::msg::PointCloud2 cloud;

  inline void vectorTfToEigen(tf2::Vector3 & tf_vec, Eigen::Vector3f & eigen_vec)
  {
    eigen_vec(0) = tf_vec[0];
    eigen_vec(1) = tf_vec[1];
    eigen_vec(2) = tf_vec[2];
  }

  inline void transformPoint(float & x, float & y, float & z)
  {
    Eigen::Vector3f p = Eigen::Vector3f(x, y, z);
    if (!config_.fixed_frame.empty()) {
      p = tf_matrix_to_fixed_ * p;
    }
    if (!config_.target_frame.empty()) {
      p = tf_matrix_to_target_ * p;
    }
    x = p.x();
    y = p.y();
    z = p.z();
  }

  inline bool pointInRange(float range)
  {
    return range >= config_.min_range && range <= config_.max_range;
  }

  Config config_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  Eigen::Affine3f tf_matrix_to_fixed_;
  Eigen::Affine3f tf_matrix_to_target_;
  std::string sensor_frame_;
};
}  // namespace velodyne_rawdata

#endif  // VELODYNE_POINTCLOUD__DATACONTAINERBASE_HPP_
