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

#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include <velodyne_pointcloud/organized_cloudXYZIR.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh, std::string const & node_name):
    data_(new velodyne_rawdata::RawData()), first_rcfg_call(true),
    diagnostics_(node, private_nh, node_name)
  {

    boost::optional<velodyne_pointcloud::Calibration> calibration = data_->setup(private_nh);
    if(calibration)
    {
        ROS_DEBUG_STREAM("Calibration file loaded.");
        config_.num_lasers = static_cast<uint16_t>(calibration.get().num_lasers);
    }
    else
    {
        ROS_ERROR_STREAM("Could not load calibration file!");
    }

    if(config_.organize_cloud)
    {
      container_ptr_ = boost::shared_ptr<OrganizedCloudXYZIR>(
          new OrganizedCloudXYZIR(config_.max_range, config_.min_range,
              config_.target_frame, config_.fixed_frame,
              config_.num_lasers, data_->scansPerPacket()));
    }
    else
    {
      container_ptr_ = boost::shared_ptr<PointcloudXYZIR>(
          new PointcloudXYZIR(config_.max_range, config_.min_range,
              config_.target_frame, config_.fixed_frame,
              data_->scansPerPacket()));
    }

    // Initilize all parameters used for data block azimuth difference calculation
    last_azimuth_ = -1;
    current_azimuth_diff_ = -1;
    prev_azimuth_diff_ = -1;

    container_ptr_->last_azimuth_corrected = -1;
    container_ptr_->current_corrected_azimuth_diff = -1;
    container_ptr_->prev_corrected_azimuth_diff = -1;

    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));

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

  void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
    ROS_INFO("Reconfigure Request");
    data_->setParameters(config.min_range, config.max_range, config.view_direction,
                         config.view_width);
    config_.fixed_frame = config.fixed_frame;
    config_.target_frame = config.target_frame;
    config_.min_range = config.min_range;
    config_.max_range = config.max_range;

    if(first_rcfg_call || config.organize_cloud != config_.organize_cloud){
        first_rcfg_call = false;
        config_.organize_cloud = config.organize_cloud;
        if(config_.organize_cloud) // TODO only on change
        {
            ROS_INFO_STREAM("Using the organized cloud format...");
            container_ptr_ = boost::shared_ptr<OrganizedCloudXYZIR>(
                new OrganizedCloudXYZIR(config_.max_range, config_.min_range,
                    config_.target_frame, config_.fixed_frame,
                    config_.num_lasers, data_->scansPerPacket()));
        }
        else
        {
            container_ptr_ = boost::shared_ptr<PointcloudXYZIR>(
                new PointcloudXYZIR(config_.max_range, config_.min_range,
                    config_.target_frame, config_.fixed_frame,
                    data_->scansPerPacket()));
        }
    }

    container_ptr_->configure(config_.max_range, config_.min_range, config_.fixed_frame, config_.target_frame);

  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);
    // allocate a point cloud with same time and frame ID as raw data
    container_ptr_->setup(scanMsg);

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
        velodyne_msgs::VelodynePacket tmp_packet = scanMsg->packets[i];

        // Extract base rotation of first block in packet
        std::size_t azimuth_data_pos = 100*0+2;
        int azimuth = *( (u_int16_t*) (&tmp_packet.data[azimuth_data_pos]));

        //if first packet in scan, there is no "valid" last_azimuth_
        if (last_azimuth_ == -1)
        {
            last_azimuth_ = azimuth;
            continue;
        }
        if (current_azimuth_diff_ == -1)
        {
            current_azimuth_diff_ = azimuth - last_azimuth_;
            continue;
        }
        if (prev_azimuth_diff_ == -1)
        {
            prev_azimuth_diff_ = current_azimuth_diff_;
            continue;
        }
        current_azimuth_diff_ = azimuth - last_azimuth_;

        adjusted_vel_msg_ptr->packets.push_back(tmp_packet);

        if ((current_azimuth_diff_ < 0) && (prev_azimuth_diff_ >= 0))
        {
            // Current and previous azimuth differences have the same sign indicating the current packet was taken at 0 deg
            adjusted_vel_msg_ptr->header.stamp = tmp_packet.stamp;

            // allocate a point cloud with same time and frame ID as raw data
            container_ptr_->setup(adjusted_vel_msg_ptr);
            container_ptr_->offloadResidualPoint();

            // process each packet
            for (size_t j = 0; j < adjusted_vel_msg_ptr->packets.size(); ++j)
            {
              data_->unpack(adjusted_vel_msg_ptr->packets[j], *container_ptr_);
            }

            // Clear all the converted packets
            adjusted_vel_msg_ptr->packets.clear();
        }
        last_azimuth_ = azimuth;
        prev_azimuth_diff_ = current_azimuth_diff_;
    }

    // publish the accumulated cloud message
    diag_topic_->tick(adjusted_vel_msg_ptr->header.stamp);
    diagnostics_.update();
    output_.publish(container_ptr_->finishCloud());
  }

} // namespace velodyne_pointcloud
