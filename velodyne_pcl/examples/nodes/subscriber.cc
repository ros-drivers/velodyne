#include <ros/ros.h>

#include <velodyne_pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <boost/foreach.hpp>

typedef velodyne_pcl::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

void callback(const VPointCloud::ConstPtr& msg) {
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  //BOOST_FOREACH (const VPoint& pt, msg->points) {
    //printf ("\t(%f, %f, %f, %f, %i)\n", pt.x, pt.y, pt.z, pt.intensity, pt.ring);
  //}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "velodyne_example_sub");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<VPointCloud>("velodyne/pcl_points2", 1, callback);
  ros::spin();
}

