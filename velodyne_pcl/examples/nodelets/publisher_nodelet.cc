#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <velodyne/data_xyz.h>
#include <velodyne/ring_sequence.h>
#include <velodyne/data_base.h>
#include <velodyne_pcl/point_types.h>

#include <pcl_ros/point_cloud.h>

namespace velodyne_pcl {

  class ExamplePublisherNodelet: public nodelet::Nodelet {

    typedef velodyne_pcl::PointXYZIR VPoint;
    typedef pcl::PointCloud<VPoint> VPointCloud;

    private:

    virtual void onInit();
    void processXYZ(const Velodyne::xyz_scans_t &scan,
                    ros::Time stamp,
                    const std::string &frame_id);

    boost::shared_ptr<Velodyne::DataXYZ> data_;
    ros::Subscriber velodyne_sub_;
    ros::Publisher velodyne_pub_;

    VPointCloud::Ptr vpc_;
    uint16_t packet_count_;
    uint16_t total_packets_;
  };

  void ExamplePublisherNodelet::onInit() {

    ros::NodeHandle node = getNodeHandle();

    data_.reset(new Velodyne::DataXYZ());
    data_->getParams();
    if (0 != data_->setup())
      return;

    velodyne_pub_ = node.advertise<VPointCloud>("velodyne/pcl_points2", 1);

    velodyne_sub_ =
      data_->subscribe(node, "velodyne/packets", 1,
                       boost::bind(&ExamplePublisherNodelet::processXYZ, this, _1, _2, _3),
                       ros::TransportHints().tcpNoDelay(true));

    packet_count_ = 0;
    total_packets_ = velodyne_msgs::VelodyneScan::PACKETS_PER_REVOLUTION;

    vpc_.reset(new VPointCloud);
    vpc_->height = velodyne::N_LASERS;
    vpc_->width = total_packets_ * Velodyne::SCANS_PER_PACKET / vpc_->height;

  }

  void ExamplePublisherNodelet::processXYZ(const Velodyne::xyz_scans_t &scan,
                                 ros::Time stamp,
                                 const std::string &frame_id) {

    if (velodyne_pub_.getNumSubscribers() == 0)        
      return;                                   

    vpc_->header.stamp = stamp;
    vpc_->header.frame_id = frame_id;

    // fill in point values
    size_t npoints = scan.size();
    for (size_t i = 0; i < npoints; i++) {
      VPoint p;
      p.x = scan[i].x;
      p.y = scan[i].y;
      p.z = scan[i].z;
      p.intensity = scan[i].intensity;
      p.ring = velodyne::LASER_RING[scan[i].laser_number];
      vpc_->points.push_back(p);
    }

    packet_count_++;

    if (packet_count_ == total_packets_) {
      NODELET_INFO("Publishing %i packets", packet_count_);
      velodyne_pub_.publish(vpc_);
      vpc_->points.clear();
      packet_count_ = 0;
    }
  }

} // namespace velodyne_pcl

// Register this plugin with pluginlib. Names must match nodelets.xml.
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_pcl, ExamplePublisherNodelet,
                        velodyne_pcl::ExamplePublisherNodelet, nodelet::Nodelet);
