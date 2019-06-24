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

*/

#include "velodyne_pointcloud/transform.h"

#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Transform::Transform(const rclcpp::NodeOptions & options) :
    rclcpp::Node("transform_node", options),
    buffer_(this->get_clock()),
    tf2_filter_(velodyne_scan_, buffer_, config_.frame_id, 10, 0),
    data_(new velodyne_rawdata::RawData(options))
  {
    // Read calibration.
    data_->setup();

    // advertise output point cloud (before subscribing to input data)
    output_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points", 10);
    
    // subscribe to VelodyneScan packets using transform filter
    velodyne_scan_.subscribe(this->shared_from_this(), "velodyne_packets");
    
    tf2_filter_.registerCallback(std::bind(&Transform::processScan, this, std::placeholders::_1));

    // Diagnostics
    diagnostics_.setHardwareID("Velodyne Transform");
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

  /** @brief Callback for raw scan messages.
   *
   *  @pre TF message filter has already waited until the transform to
   *       the configured @c frame_id can succeed.
   */
  void
  Transform::processScan(const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr &scanMsg)
  {
    //if (output_.getNumSubscribers() == 0)         // no one listening?
    //  return;                                     // avoid much work

    // allocate an output point cloud with same time as raw data
    velodyne_rawdata::VPointCloud::Ptr outMsg(new velodyne_rawdata::VPointCloud());
    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg->header.frame_id = config_.frame_id;
    outMsg->height = 1;

    // process each packet provided by the driver
    for (size_t next = 0; next < scanMsg->packets.size(); ++next)
      {
        // clear input point cloud to handle this packet
        inPc_.pc->points.clear();
        inPc_.pc->width = 0;
        inPc_.pc->height = 1;
        std_msgs::msg::Header header;
        header.stamp = scanMsg->packets[next].stamp;
        header.frame_id = scanMsg->header.frame_id;
        pcl_conversions::toPCL(header, inPc_.pc->header);

        // unpack the raw data
        data_->unpack(scanMsg->packets[next], inPc_);

        // clear transform point cloud for this packet
        tfPc_.points.clear();           // is this needed?
        tfPc_.width = 0;
        tfPc_.height = 1;
        header.stamp = scanMsg->packets[next].stamp;
        pcl_conversions::toPCL(header, tfPc_.header);
        tfPc_.header.frame_id = config_.frame_id;

        // transform the packet point cloud into the target frame
        try
          {
            RCLCPP_DEBUG(this->get_logger(), "correcting distortion relative to %d", config_.frame_id.c_str());
            // stamp in the header is microseconds unix time.
            rclcpp::Time stamp(outMsg->header.stamp / 1000000, (outMsg->header.stamp % 1000000)*1000);
            pcl_ros::transformPointCloud(inPc_.pc->header.frame_id,
                                         stamp,
                                         *(inPc_.pc),
                                         config_.frame_id,
                                         tfPc_,
                                         buffer_);
          }
        catch (tf2::TransformException &ex)
          {
            // only log tf error once every 100 times
          RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            continue;                   // skip this packet
          }

        // append transformed packet data to end of output message
        outMsg->points.insert(outMsg->points.end(),
                             tfPc_.points.begin(),
                             tfPc_.points.end());
        outMsg->width += tfPc_.points.size();
      }

    // publish the accumulated cloud message
    RCLCPP_DEBUG(this->get_logger(), "Publishing %d Velodyne points, time: %s", outMsg->height * outMsg->width
                     , outMsg->header.stamp);
    output_->publish(outMsg);
    diag_topic_->tick(scanMsg->header.stamp);
    diagnostics_.update();
  }

} // namespace velodyne_pointcloud

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(velodyne_pointcloud::Transform)
