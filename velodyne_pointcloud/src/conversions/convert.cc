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
#include <velodyne_pointcloud/pointcloudXYZIRT.h>
#include <velodyne_pointcloud/organized_cloudXYZIR.h>
#include <velodyne_pointcloud/organized_cloudXYZIRT.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh, std::string const & node_name):
    data_(new velodyne_rawdata::RawData()), first_rcfg_call(true),
    diagnostics_(node, private_nh, node_name)
  {
    // Get startup parameters
    private_nh.param<std::string>("fixed_frame", config_.fixed_frame, "velodyne");
    private_nh.param<std::string>("target_Frame", config_.target_frame, "velodyne");
    private_nh.param<double>("min_range", config_.min_range, 10.0);
    private_nh.param<double>("max_range", config_.max_range, 200.0);
    private_nh.param<bool>("organize_cloud", config_.organize_cloud, false);

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

    // container selection and check
    std::string container_name;
    container_configured_ = private_nh.getParam("container", container_name);
    private_nh.param<bool>("organize_cloud", config_.organize_cloud, false);
    if(!container_configured_)
    {
      std::string default_container_name, default_organized_container_name;
      std::stringstream container_list;
      for (const auto& pair : container_names_)
      {
        container_list << pair.first << ", ";
      }

      if(config_.organize_cloud)
      {
        ROS_WARN_STREAM("The parameter \"organize_cloud\" is deprecated, please use the \"container\" param "
                        "with one of the organized containers!");
        container_name = default_organized_container_.first;
        container_ptr_ = getContainer(default_organized_container_.second);
      }
      else
      {
        container_name = default_container_.first;
        container_ptr_ = getContainer(default_container_.second);
      }
      ROS_WARN_STREAM("Using default container \"" << container_name << "\"!");
      ROS_WARN_STREAM("Please use the \"container\" param with one of the following container names: " << container_list.str());
    }
    else
    {
      std::map<std::string, Container>::iterator container_it_ = container_names_.find(container_name);
      if(container_it_ == container_names_.end())
      {
        ROS_ERROR_STREAM("Container with name \"" << container_name << "\" does not exist!");
        std::stringstream container_list;
        for (const auto& pair : container_names_)
        {
          container_list << pair.first << ", ";
        }
        ROS_ERROR_STREAM("Use one of the following container names: " << container_list.str());
        exit(EXIT_FAILURE);
      }
      container_ptr_ = getContainer(container_it_->second);
      ROS_INFO_STREAM("Using the \"" << container_it_->first << "\" container.");
    }


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

  boost::shared_ptr<velodyne_rawdata::DataContainerBase> Convert::getContainer(uint8_t container_id)
  {
    switch (container_id)
    {
      case Container::PointCloudXYZPIR:
        return boost::shared_ptr<PointcloudXYZIR>(
            new PointcloudXYZIR("p", config_.max_range, config_.min_range,
                                config_.target_frame, config_.fixed_frame,
                                data_->scansPerPacket()));
      case Container::PointCloudXYZIR:
        return boost::shared_ptr<PointcloudXYZIR>(
            new PointcloudXYZIR(config_.max_range, config_.min_range,
                                config_.target_frame, config_.fixed_frame,
                                data_->scansPerPacket()));
      case Container::PointCloudXYZIRT:
        return boost::shared_ptr<PointcloudXYZIRT>(
            new PointcloudXYZIRT(config_.max_range, config_.min_range,
                                 config_.target_frame, config_.fixed_frame,
                                 data_->scansPerPacket()));
      case Container::OrganizedPointCloudXYZIR:
        return boost::shared_ptr<OrganizedCloudXYZIR>(
            new OrganizedCloudXYZIR(config_.max_range, config_.min_range, config_.target_frame, config_.fixed_frame,
                                    config_.num_lasers, data_->scansPerPacket()));
      case Container::OrganizedPointCloudXYZIRT:
        return boost::shared_ptr<OrganizedCloudXYZIRT>(
            new OrganizedCloudXYZIRT(config_.max_range, config_.min_range, config_.target_frame, config_.fixed_frame,
                                     config_.num_lasers, data_->scansPerPacket()));
      default:
        ROS_FATAL_STREAM("Container for id " << container_id << " does not exist! Exit node!");
        exit(EXIT_FAILURE);
    }
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

        if(!container_configured_)
        {
          std::string container_name;
          if(config_.organize_cloud)
          {
            ROS_WARN_STREAM("The parameter \"organize_cloud\" is deprecated, please use the \"container\" param "
                            "with one of the organized containers!");
            container_name = default_organized_container_.first;
            container_ptr_ = getContainer(default_organized_container_.second);
          }
          else
          {
            container_name = default_container_.first;
            container_ptr_ = getContainer(default_container_.second);
          }
          ROS_INFO_STREAM("Using the \"" << container_name << "\" container.");
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
      data_->unpack(scanMsg->packets[i], *container_ptr_, scanMsg->header.stamp);
    }

    // publish the accumulated cloud message
    diag_topic_->tick(scanMsg->header.stamp);
    diagnostics_.update();
    output_.publish(container_ptr_->finishCloud());
  }

} // namespace velodyne_pointcloud
