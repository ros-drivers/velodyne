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

#include "velodyne_pointcloud/transform.h"

#include <velodyne_pointcloud/pointcloudXYZIRT.h>
#include <velodyne_pointcloud/organized_cloudXYZIRT.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh, std::string const & node_name):
    data_(new velodyne_rawdata::RawData),
    first_rcfg_call(true),
    diagnostics_(node, private_nh, node_name)
  {
    const std::string marker_frame_id{private_nh.param<std::string>("target_frame", "velodyne")};
    const std::string marker_ns{private_nh.param<std::string>("model", "64E")};
    const float max_range{private_nh.param<float>("max_range", 200.0)};

    boost::optional<velodyne_pointcloud::Calibration> calibration = data_->setup(private_nh);

    if (calibration) {
      ROS_DEBUG_STREAM("Calibration file loaded.");
      config_.num_lasers = static_cast<uint16_t>(calibration.get().num_lasers);
      // publish RViz markers
      visualization_msgs::MarkerArray marker_array;
      size_t marker_id{0};
      geometry_msgs::Point origin_point;
      origin_point.x = origin_point.y = origin_point.z = 0.0;
      // white for 128 beam
      float marker_color_r{1.0};
      float marker_color_g{1.0};
      float marker_color_b{1.0};
      switch (calibration.get().num_lasers) {
        case 64: marker_color_g = 0.0; break;
        case 32: marker_color_r = 0.0; break;
        case 16: marker_color_b = 0.0; break;
      }
      geometry_msgs::Point other_point;
      other_point.y = 0.0;
      for (const auto laser_correction : calibration->laser_corrections) {
        // front point
        other_point.x = laser_correction.cos_vert_correction * max_range;
        other_point.z = laser_correction.sin_vert_correction * max_range;
        marker_array.markers.push_back(
          create_laser_marker(marker_frame_id,
                              marker_ns,
                              marker_id++,
                              marker_color_r,
                              marker_color_g,
                              marker_color_b,
                              origin_point,
                              other_point));
        // rear point
        other_point.x = -other_point.x;
        marker_array.markers.push_back(
          create_laser_marker(marker_frame_id,
                              marker_ns,
                              marker_id++,
                              marker_color_r,
                              marker_color_g,
                              marker_color_b,
                              origin_point,
                              other_point));
      }
      this->rviz_ = private_nh.advertise<visualization_msgs::MarkerArray>(
          "lidar_beams", 1, true);
      this->rviz_.publish(marker_array);
    }
    else {
      ROS_ERROR_STREAM("Could not load calibration file!");
    }

    // advertise output point cloud (before subscribing to input data)
    output_ = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

    srv_ = boost::make_shared<dynamic_reconfigure::Server<TransformNodeCfg>> (private_nh);
    dynamic_reconfigure::Server<TransformNodeCfg>::CallbackType f;
    f = boost::bind (&Transform::reconfigure_callback, this, _1, _2);
    srv_->setCallback (f);

    velodyne_scan_ = node.subscribe("velodyne_packets", 10, &Transform::processScan, this);

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

  void Transform::reconfigure_callback(
      velodyne_pointcloud::TransformNodeConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM("Reconfigure request.");
    data_->setParameters(config.min_range, config.max_range,
                         config.view_direction, config.view_width);
    config_.target_frame = config.target_frame;
    config_.fixed_frame = config.fixed_frame;
    ROS_INFO_STREAM("Target frame ID now: " << config_.target_frame);
    ROS_INFO_STREAM("Fixed frame ID now: " << config_.fixed_frame);
    config_.min_range = config.min_range;
    config_.max_range = config.max_range;

    boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);

    if(first_rcfg_call || config.organize_cloud != config_.organize_cloud){
      first_rcfg_call = false;
      config_.organize_cloud = config.organize_cloud;
      if(config_.organize_cloud)
      {
        ROS_INFO_STREAM("Using the organized cloud format...");
        container_ptr = boost::shared_ptr<OrganizedCloudXYZIRT>(
            new OrganizedCloudXYZIRT(config_.max_range, config_.min_range,
                                    config_.target_frame, config_.fixed_frame,
                                    config_.num_lasers, data_->scansPerPacket()));
      }
      else
      {
        container_ptr = boost::shared_ptr<PointcloudXYZIRT>(
            new PointcloudXYZIRT(config_.max_range, config_.min_range,
                                config_.target_frame, config_.fixed_frame,
                                data_->scansPerPacket()));
      }
    }
    container_ptr->configure(config_.max_range, config_.min_range, config_.fixed_frame, config_.target_frame);
  }

  /** @brief Callback for raw scan messages.
   *
   *  @pre TF message filter has already waited until the transform to
   *       the configured @c frame_id can succeed.
   */
  void
    Transform::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)      // no one listening?
      return;                                     // avoid much work

    boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);

    // allocate a point cloud with same time and frame ID as raw data
    container_ptr->setup(scanMsg);

    // sufficient to calculate single transform for whole scan
    if(!container_ptr->computeTransformToTarget(scanMsg->header.stamp))
    {
      // target frame not available
      return;
    }

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      // calculate individual transform for each packet to account for ego
      // during one rotation of the velodyne sensor
      if(!container_ptr->computeTransformToFixed(scanMsg->packets[i].stamp))
      {
        // fixed frame not available
        return;
      }
      data_->unpack(scanMsg->packets[i], *container_ptr, scanMsg->header.stamp);
    }
    // publish the accumulated cloud message
    output_.publish(container_ptr->finishCloud());

    diag_topic_->tick(scanMsg->header.stamp);
    diagnostics_.update();
  }

  visualization_msgs::Marker Transform::create_laser_marker(const std::string & frame_id,
                                                            const std::string & ns,
                                                            const size_t & marker_id,
                                                            const float & marker_color_r,
                                                            const float & marker_color_g,
                                                            const float & marker_color_b,
                                                            const geometry_msgs::Point & pt1,
                                                            const geometry_msgs::Point & pt2) const {
      visualization_msgs::Marker line_marker;
      line_marker.ns = ns;
      line_marker.header.frame_id = frame_id;
      line_marker.header.stamp = ros::Time(0);
      line_marker.id = marker_id;
      line_marker.type = visualization_msgs::Marker::LINE_STRIP;
      line_marker.action = visualization_msgs::Marker::ADD;
      line_marker.pose.position.x = line_marker.pose.position.y = line_marker.pose.position.z = 0.0;
      line_marker.pose.orientation.x = line_marker.pose.orientation.y = line_marker.pose.orientation.z = 0.0;
      line_marker.pose.orientation.w = 1.0;
      line_marker.scale.x = 0.005;
      line_marker.color.a = 1;
      line_marker.color.r = marker_color_r;
      line_marker.color.g = marker_color_g;
      line_marker.color.b = marker_color_b;
      line_marker.lifetime = ros::Duration(0.0);  // lifetime is forever
      line_marker.frame_locked = true;            // re-update every frame change
      line_marker.points.push_back(pt1);
      line_marker.points.push_back(pt2);
      return line_marker;
  }

} // namespace velodyne_pointcloud
