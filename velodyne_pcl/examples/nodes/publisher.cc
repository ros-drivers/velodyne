#include <ros/ros.h>

#include <velodyne/data_xyz.h>
#include <velodyne/ring_sequence.h>
#include <velodyne/data_base.h>
#include <velodyne_pcl/point_types.h>

#include <pcl_ros/point_cloud.h>

typedef velodyne_pcl::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

namespace {
  boost::shared_ptr<Velodyne::DataXYZ> data_;
  VPointCloud::Ptr vpc_;
  ros::Publisher velodyne_pub_;
  ros::Subscriber velodyne_sub_;
}

void processXYZ(const Velodyne::xyz_scans_t &scan,
                ros::Time stamp,
                const std::string &frame_id) {

  if (velodyne_pub_.getNumSubscribers() == 0)        
    return;                                   

  vpc_->header.stamp = stamp;
  vpc_->header.frame_id = frame_id;

  // fill in point values
  size_t npoints = scan.size();
  for (size_t i = 0; i < npoints; i++) {
    VPoint &p = vpc_->points[i];
    p.x = scan[i].x;
    p.y = scan[i].y;
    p.z = scan[i].z;
    p.intensity = scan[i].intensity;
    p.ring = velodyne::LASER_RING[scan[i].laser_number];
  }

  velodyne_pub_.publish(vpc_);  
}

int main(int argc, char** argv) {

  ros::init (argc, argv, "velodyne_example_pub");
  ros::NodeHandle nh;
  velodyne_pub_ = nh.advertise<VPointCloud> ("velodyne/pcl_points2", 1);

  velodyne_sub_ =
    data_->subscribe(nh, "velodyne/packets", 1,
                     boost::bind(&processXYZ, _1, _2, _3),
                     ros::TransportHints().tcpNoDelay(true));

  vpc_.reset(new VPointCloud);
  vpc_->points.reserve(velodyne_msgs::VelodyneScan::PACKETS_PER_REVOLUTION * Velodyne::SCANS_PER_PACKET);
  vpc_->height = 64;
  vpc_->width = velodyne_msgs::VelodyneScan::PACKETS_PER_REVOLUTION * Velodyne::SCANS_PER_PACKET / vpc_->height;

  ros::spin();

}
