#ifndef __DATACONTAINERBASE_H
#define __DATACONTAINERBASE_H

#include <tf/transform_listener.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/Dense>
#include <cstdarg>

namespace velodyne_rawdata
{
  class DataContainerBase
  {
   public:
    DataContainerBase(
        const double max_range, const double min_range,
        const std::string& target_frame, const std::string& fixed_frame,
        const unsigned int init_width, const unsigned int init_height,
        const bool is_dense, const unsigned int scans_per_packet, int fields, ...)
        : config_(max_range, min_range, target_frame, fixed_frame,
          init_width, init_height, is_dense, scans_per_packet) {

      va_list vl;
      cloud.fields.clear();
      cloud.fields.reserve(fields);
      va_start(vl, fields);
      int offset = 0;
      for (int i = 0; i < fields; ++i) {
        // Create the corresponding PointField
        std::string name(va_arg(vl, char*));
        int count(va_arg(vl, int));
        int datatype(va_arg(vl, int));
        offset = addPointField(cloud, name, count, datatype, offset);
      }
      va_end(vl);
      cloud.point_step = offset;
      cloud.row_step = cloud.width * cloud.point_step;
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

    virtual void setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg){
      cloud.header = scan_msg->header;
      cloud.data.resize(scan_msg->packets.size() * config_.scans_per_packet * cloud.point_step);
      cloud.width = config_.init_width;
      cloud.height = config_.init_height;
      cloud.is_dense = static_cast<uint8_t >(config_.is_dense);
    }

    virtual void addPoint(
        float x, float y, float z,
        const uint16_t ring, const uint16_t azimuth,
        const float distance, const float intensity) = 0;
    virtual void newLine() = 0;

    const sensor_msgs::PointCloud2& finishCloud(){
      cloud.data.resize(cloud.point_step * cloud.width * cloud.height);
      cloud.header.frame_id = config_.target_frame;
      ROS_DEBUG_STREAM("Prepared cloud width" << cloud.height * cloud.width
          << " Velodyne points, time: " << cloud.header.stamp);

      return cloud;
    }

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

    sensor_msgs::PointCloud2 cloud;

   protected:

    inline void vectorTfToEigen(tf::Vector3& tf_vec, Eigen::Vector3d& eigen_vec){
      eigen_vec(0) = tf_vec[0];
      eigen_vec(1) = tf_vec[1];
      eigen_vec(2) = tf_vec[2];
    }

    bool computeTransformation(const ros::Time& time){
      tf::StampedTransform transform;
      try{
        tf_ptr->lookupTransform(config_.target_frame, cloud.header.frame_id, time, transform);
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

    void transformPoint(float& x, float& y, float& z)
    {
      Eigen::Vector3d p = transformation * Eigen::Vector3d(x, y, z);
      x = p.x(); y = p.y(); z = p.z();
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
