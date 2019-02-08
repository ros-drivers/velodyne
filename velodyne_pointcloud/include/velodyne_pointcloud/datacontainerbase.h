#ifndef __DATACONTAINERBASE_H
#define __DATACONTAINERBASE_H

#include <pcl_ros/point_cloud.h>
#include <velodyne_pointcloud/point_types.h>
#include <tf/transform_listener.h>
#include <velodyne_msgs/VelodyneScan.h>

namespace velodyne_rawdata
{
  // Shorthand typedefs for point cloud representations
  typedef velodyne_pointcloud::PointXYZIR VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  class DataContainerBase
  {
   public:
    DataContainerBase(
        const double max_range, const double min_range,
        const std::string& target_frame, const std::string& fixed_frame,
        const unsigned int init_width, const unsigned int init_height,
        const bool is_dense, const unsigned int scans_per_packet)
        : config_(max_range, min_range, target_frame, fixed_frame,
          init_width, init_height, is_dense, scans_per_packet), pc(new VPointCloud) {

      if(config_.transform)
      {
        tf_ptr = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener);
      }

    }

    struct Config {
      double max_range;                ///< maximum range to publish
      double min_range;                ///< minimum range to publish
      std::string target_frame;        ///< target frame to transform a point
      std::string fixed_frame;         ///< fixed frame used for transform
      unsigned int init_width;
      unsigned int init_height;
      bool is_dense;
      unsigned int scans_per_packet;
      bool transform;                  ///< enable / disable transform points

      Config(
          double max_range, double min_range,
          std::string target_frame, std::string fixed_frame,
          unsigned int init_width, unsigned int init_height,
          bool is_dense, unsigned int scans_per_packet)
            : max_range(max_range), min_range(min_range),
              target_frame(target_frame), fixed_frame(fixed_frame),
              transform(fixed_frame!=target_frame),
              init_width(init_width), init_height(init_height),
              is_dense(is_dense), scans_per_packet(scans_per_packet)
          {
            ROS_INFO_STREAM("Initialized container with "
                << "min_range: "<< min_range << ", max_range: " << max_range
                << ", target_frame: " << target_frame << ", fixed_frame: " << fixed_frame
                << ", init_with: " << init_width << ", init_height: " << init_height
                << ", is_dense: " << is_dense << ", scans_per_packet: " << scans_per_packet);
          }
    };

    void setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg){
      pc = VPointCloud::Ptr(new VPointCloud);
      pc->header.stamp = pcl_conversions::toPCL(scan_msg->header.stamp);
      pc->header.frame_id = scan_msg->header.frame_id;
      pc->points.reserve(scan_msg->packets.size() * config_.scans_per_packet);
      pc->width = config_.init_width;
      pc->height = config_.init_height;
      pc->is_dense = config_.is_dense;
    }

    virtual void addPoint(
        const float& x, const float& y, const float& z,
        const uint16_t& ring, const uint16_t& azimuth,
        const float& distance, const float& intensity) = 0;
    virtual void newLine() = 0;

    void configure(const double max_range, const double min_range,
        const std::string fixed_frame, const std::string target_frame) {
      config_.max_range = max_range;
      config_.min_range = min_range;
      config_.fixed_frame = fixed_frame;
      config_.target_frame = target_frame;
      config_.transform = fixed_frame != target_frame;
      if(config_.transform)
      {
        tf_ptr = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener);
      }
    }

    VPointCloud::Ptr pc;

   protected:

    inline void vectorTfToEigen(tf::Vector3& tf_vec, Eigen::Vector3d& eigen_vec){
      eigen_vec(0) = tf_vec[0];
      eigen_vec(1) = tf_vec[1];
      eigen_vec(2) = tf_vec[2];
    }

    bool computeTransformation(const ros::Time& time){
      tf::StampedTransform transform;
      try{
        tf_ptr->lookupTransform(config_.target_frame, pc->header.frame_id, time, transform);
      }
      catch (tf::LookupException &e){
        ROS_ERROR ("%s", e.what ());
        return false;
      }
      catch (tf::ExtrapolationException &e){
        ROS_ERROR ("%s", e.what ());
        return false;
      }

      tf::Quaternion quaternion = transform.getRotation ();
      Eigen::Quaterniond rotation (
          quaternion.w (),
          quaternion.x (),
          quaternion.y (),
          quaternion.z ()
      );

      tf::Vector3 origin = transform.getOrigin ();
      Eigen::Vector3d eigen_origin;
      vectorTfToEigen(origin, eigen_origin);
      Eigen::Translation3d translation ( eigen_origin );
      transformation = translation * rotation;
      return true;
    }

    inline bool transformPoint(velodyne_rawdata::VPoint& point)
    {
      Eigen::Vector3d vec(point.x, point.y, point.z), out_vec;
      out_vec = transformation * vec;
      point.x = out_vec(0);
      point.y = out_vec(1);
      point.z = out_vec(2);
    }

    inline bool pointInRange(float range)
    {
      return (range >= config_.min_range
          && range <= config_.max_range);
    }

    Config config_;

    boost::shared_ptr<tf::TransformListener> tf_ptr; ///< transform listener

    Eigen::Affine3d transformation;
};
}
#endif //__DATACONTAINERBASE_H
