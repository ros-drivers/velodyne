/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include <cmath>
#include <functional>
#include <memory>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <velodyne_msgs/msg/velodyne_scan.hpp>
#include <velodyne_pointcloud/convert.h>
#include <velodyne_pointcloud/organized_cloudXYZIR.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include <velodyne_pointcloud/rawdata.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert() : rclcpp::Node("convert"),
    data_(new velodyne_rawdata::RawData()), tf_buffer_(this->get_clock())
  {
    // get path to angles.config file for this device
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

    config_.organize_cloud = this->declare_parameter("organize_cloud", false);

    RCLCPP_INFO(this->get_logger(), "correction angles: %s", calibrationFile.c_str());

    int success = data_->setup(calibrationFile);
    if (success >= 0)
      {
        RCLCPP_DEBUG(get_logger(), "Calibration file loaded.");
        config_.num_lasers = success;
      }
    else
      {
        RCLCPP_ERROR(get_logger(), "Could not load calibration file!");
      }

    config_.target_frame = config_.fixed_frame = "velodyne";

    if (config_.organize_cloud)
      {
        container_ptr_ = std::shared_ptr<OrganizedCloudXYZIR>(
          new OrganizedCloudXYZIR(config_.max_range, config_.min_range,
            config_.target_frame, config_.fixed_frame,
            config_.num_lasers, data_->scansPerPacket(), tf_buffer_));
      }
    else
      {
        container_ptr_ = std::shared_ptr<PointcloudXYZIR>(
          new PointcloudXYZIR(config_.max_range, config_.min_range,
            config_.target_frame, config_.fixed_frame,
            data_->scansPerPacket(), tf_buffer_));
      }

    // advertise output point cloud (before subscribing to input data)
    output_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points", 10);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      this->create_subscription<velodyne_msgs::msg::VelodyneScan>("velodyne_packets", rclcpp::QoS(10),
                                                                  std::bind(&Convert::processScan, this, std::placeholders::_1));

    // Diagnostics
    //diagnostics_.setHardwareID("Velodyne Convert");
    // Arbitrary frequencies since we don't know which RPM is used, and are only
    // concerned about monitoring the frequency.
    diag_min_freq_ = 2.0;
    diag_max_freq_ = 20.0;
    // using namespace diagnostic_updater;
    // diag_topic_.reset(new TopicDiagnostic("velodyne_points", diagnostics_,
    //                                    FrequencyStatusParam(&diag_min_freq_,
    //                                                         &diag_max_freq_,
    //                                                         0.1, 10),
    //                                    TimeStampStatusParam()));

    data_->setParameters(config_.min_range, config_.max_range, config_.view_direction, config_.view_width);
    container_ptr_->configure(config_.max_range, config_.min_range, config_.fixed_frame, config_.target_frame);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::msg::VelodyneScan::SharedPtr scanMsg)
  {
    if (output_->get_subscription_count() == 0)   // no one listening?
      return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    container_ptr_->setup(scanMsg);

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
      {
        data_->unpack(scanMsg->packets[i], *container_ptr_);
      }

    // publish the accumulated cloud message
    //diag_topic_->tick(scanMsg->header.stamp);
    //diagnostics_.update();
    output_->publish(container_ptr_->finishCloud());
  }

} // namespace velodyne_pointcloud
