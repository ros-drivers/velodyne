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

#include "velodyne_pointcloud/convert.h"

#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(const rclcpp::NodeOptions & options) : rclcpp::Node("convert_node", options),
    data_(new velodyne_rawdata::RawData(options))
  {
    data_->setup();


    // advertise output point cloud (before subscribing to input data)
    output_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points", 10);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      this->create_subscription("velodyne_packets", 10, std::bind(
                     &Convert::processScan, this);

    // Diagnostics
    diagnostics_.setHardwareID("Velodyne Convert");
    // Arbitrary frequencies since we don't know which RPM is used, and are only
    // concerned about monitoring the frequency.
    diag_min_freq_ = 2.0;
    diag_max_freq_ = 20.0;
    using namespace diagnostic_updater;
    diag_topic_.reset(new TopicDiagnostic("velodyne_points", diagnostics_,
                                       FrequencyStatusParam(&diag_min_freq_,
                                                            &diag_max_freq_,
                                                            0.1, 10),
                                       TimeStampStatusParam()));
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr &scanMsg)
  {
    // Not supported yet
    //if (output_.getNumSubscribers() == 0)         // no one listening?
    //  return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    PointcloudXYZIR outMsg;
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    outMsg.pc->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg.pc->header.frame_id = scanMsg->header.frame_id;
    outMsg.pc->height = 1;

    outMsg.pc->points.reserve(scanMsg->packets.size() * data_->scansPerPacket());

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      data_->unpack(scanMsg->packets[i], outMsg);
    }

    // publish the accumulated cloud message
  RCLCPP_DEBUG(this->get_logger(), "Publishing %d Velodyne points, time: %s", outMsg.pc->height * outMsg.pc->width
                     ,outMsg.pc->header.stamp);
    output_->publish(outMsg.pc);
    diag_topic_->tick(scanMsg->header.stamp);
    diagnostics_.update();
  }

} // namespace velodyne_pointcloud

#include "rclcpp_components/register_node_macro.hpp"
                              
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(velodyne_pointcloud::Convert)
