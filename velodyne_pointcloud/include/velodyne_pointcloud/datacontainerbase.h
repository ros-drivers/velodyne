#ifndef __DATACONTAINERBASE_H
#define __DATACONTAINERBASE_H

#include <pcl_ros/point_cloud.h>
#include <velodyne_pointcloud/point_types.h>
#include <tf/transform_listener.h>

namespace velodyne_rawdata
{
  // Shorthand typedefs for point cloud representations
  typedef velodyne_pointcloud::PointXYZIR VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  class DataContainerBase
  {
   public:
    DataContainerBase() : pc(new VPointCloud) {
      ROS_INFO("New container");
    }

    void reset(){pc = VPointCloud::Ptr(new VPointCloud);}
    virtual void addPoint(
        const float& x, const float& y, const float& z,
        const uint16_t& ring, const uint16_t& azimuth,
        const float& distance, const float& intensity) = 0;
    virtual void newLine() = 0;

    void setParameters(
        const double min_range,
        const double max_range,
        const std::string target_frame,
        const std::string fixed_frame,
        boost::shared_ptr<tf::TransformListener> tf_ptr
          = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener))
    {

      ROS_INFO("Config...");
      config_.max_range = max_range;
      config_.min_range = min_range;
      config_.target_frame = target_frame;
      config_.fixed_frame = fixed_frame;
      config_.tf_ptr = tf_ptr;
      config_.transform = target_frame != pc->header.frame_id;
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
        config_.tf_ptr->lookupTransform(config_.target_frame, pc->header.frame_id, time, transform);
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

    typedef struct {
      double max_range;                ///< maximum range to publish
      double min_range;                ///< minimum range to publish
      std::string target_frame;        ///< target frame to transform a point
      std::string fixed_frame;         ///< fixed frame used for transform
      boost::shared_ptr<tf::TransformListener> tf_ptr; ///< transform listener
      bool transform;                  ///< enable / disable transform points
    } Config;
    Config config_;

    Eigen::Affine3d transformation;

  };
}
#endif //__DATACONTAINERBASE_H
