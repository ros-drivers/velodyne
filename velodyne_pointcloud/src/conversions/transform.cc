/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class transforms raw Velodyne 3D LIDAR packets to PointCloud2
    in the /map frame of reference.

    @author Jack O'Quin
    @author Jesse Vera
    @author Sebastian Pütz

*/

#include <cmath>
#include <functional>
#include <memory>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <velodyne_pointcloud/organized_cloudXYZIR.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include <velodyne_pointcloud/rawdata.h>
#include "velodyne_pointcloud/transform.h"

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Transform::Transform() :
    rclcpp::Node("velodyne_transform_node"),
    velodyne_scan_(this, "velodyne_packets"), tf_buffer_(this->get_clock()), tf_filter_(velodyne_scan_, tf_buffer_, config_.target_frame, 10, 0)
  {
    std::string calibrationFile = this->declare_parameter("calibration", "");

    rcl_interfaces::msg::ParameterDescriptor min_range_desc;
    min_range_desc.name = "min_range";
    min_range_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    min_range_desc.description = "minimum range to publish";
    rcl_interfaces::msg::FloatingPointRange min_range_range;
    min_range_range.from_value = 0.1;
    min_range_range.to_value = 10.0;
    min_range_desc.floating_point_range.push_back(min_range_range);
    config_.min_range = this->declare_parameter("min_range", 0.9, min_range_desc);

    rcl_interfaces::msg::ParameterDescriptor max_range_desc;
    max_range_desc.name = "max_range";
    max_range_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    max_range_desc.description = "maximum range to publish";
    rcl_interfaces::msg::FloatingPointRange max_range_range;
    max_range_range.from_value = 0.1;
    max_range_range.to_value = 200.0;
    max_range_desc.floating_point_range.push_back(max_range_range);
    config_.max_range = this->declare_parameter("max_range", 130.0, max_range_desc);

    rcl_interfaces::msg::ParameterDescriptor view_direction_desc;
    view_direction_desc.name = "view_direction";
    view_direction_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    view_direction_desc.description = "angle defining the center of view";
    rcl_interfaces::msg::FloatingPointRange view_direction_range;
    view_direction_range.from_value = -M_PI;
    view_direction_range.to_value = M_PI;
    view_direction_desc.floating_point_range.push_back(view_direction_range);
    config_.view_direction = this->declare_parameter("view_direction", 0.0, view_direction_desc);

    rcl_interfaces::msg::ParameterDescriptor view_width_desc;
    view_width_desc.name = "view_width";
    view_width_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    view_width_desc.description = "angle defining the view width";
    rcl_interfaces::msg::FloatingPointRange view_width_range;
    view_width_range.from_value = 0.0;
    view_width_range.to_value = 2.0*M_PI;
    view_width_desc.floating_point_range.push_back(view_width_range);
    config_.view_width = this->declare_parameter("view_width", 2.0*M_PI, view_width_desc);

    config_.target_frame = this->declare_parameter("frame_id", "map");
    config_.fixed_frame = this->declare_parameter("fixed_frame", "odom");
    config_.organize_cloud = this->declare_parameter("organize_cloud", true);

    RCLCPP_INFO(this->get_logger(), "correction angles: %s", calibrationFile.c_str());

    data_ = std::make_unique<velodyne_rawdata::RawData>(calibrationFile);

    tf_ptr_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    if (config_.organize_cloud)
      {
        container_ptr_ = std::unique_ptr<OrganizedCloudXYZIR>(
          new OrganizedCloudXYZIR(config_.max_range, config_.min_range, config_.target_frame, config_.fixed_frame,
                                  data_->numLasers(), data_->scansPerPacket(), tf_buffer_));
      }
    else
      {
        container_ptr_ = std::unique_ptr<PointcloudXYZIR>(
          new PointcloudXYZIR(config_.max_range, config_.min_range,
                              config_.target_frame, config_.fixed_frame,
                              data_->scansPerPacket(), tf_buffer_));
      }

    // advertise output point cloud (before subscribing to input data)
    output_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points", 10);

    // subscribe to VelodyneScan packets using transform filter
    tf_filter_.registerCallback(std::bind(&Transform::processScan, this, std::placeholders::_1));

    // Diagnostics
    //diagnostics_.setHardwareID("Velodyne Transform");
    // Arbitrary frequencies since we don't know which RPM is used, and are only
    // concerned about monitoring the frequency.
    diag_min_freq_ = 2.0;
    diag_max_freq_ = 20.0;
    // using namespace diagnostic_updater;
    // diag_topic_.reset(new TopicDiagnostic("velodyne_points", diagnostics_,
    //                                       FrequencyStatusParam(&diag_min_freq_,
    //                                                            &diag_max_freq_,
    //                                                            0.1, 10),
    //                                       TimeStampStatusParam()));

    data_->setParameters(config_.min_range, config_.max_range, config_.view_direction, config_.view_width);
    container_ptr_->configure(config_.max_range, config_.min_range, config_.fixed_frame, config_.target_frame);
  }

  /** @brief Callback for raw scan messages.
   *
   *  @pre TF message filter has already waited until the transform to
   *       the configured @c frame_id can succeed.
   */
  void
    Transform::processScan(const std::shared_ptr<const velodyne_msgs::msg::VelodyneScan> & scanMsg)
  {
    if (output_->get_subscription_count() == 0)    // no one listening?
      {
        return;                                     // avoid much work
      }

    velodyne_msgs::msg::VelodyneScan* raw = const_cast<velodyne_msgs::msg::VelodyneScan*>(scanMsg.get());
    container_ptr_->setup(std::shared_ptr<velodyne_msgs::msg::VelodyneScan>(raw));

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
      {
        data_->unpack(scanMsg->packets[i], *container_ptr_);
      }

    // publish the accumulated cloud message
    output_->publish(container_ptr_->finishCloud());
    //diag_topic_->tick(scanMsg->header.stamp);
    //diagnostics_.update();
  }

} // namespace velodyne_pointcloud
