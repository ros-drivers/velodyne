#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <velodyne_pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <boost/foreach.hpp>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/tree_types.h>
#include <pcl/kdtree/impl/tree_types.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/impl/organized_data.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>

namespace velodyne_pcl {

  class ExampleSubscriberNodelet: public nodelet::Nodelet {

    typedef velodyne_pcl::PointXYZIR VPoint;
    typedef pcl::PointCloud<VPoint> VPointCloud;

    private:

    virtual void onInit();
    void processPointCloud(const VPointCloud::ConstPtr& msg);

    ros::Subscriber sub_;
  };

  void ExampleSubscriberNodelet::onInit() {
    ros::NodeHandle node = getNodeHandle();
    sub_ = node.subscribe<VPointCloud>("velodyne/pcl_points2", 1, 
        boost::bind(&ExampleSubscriberNodelet::processPointCloud, this, _1));
  }

  void ExampleSubscriberNodelet::processPointCloud(const VPointCloud::ConstPtr& msg) {
    NODELET_INFO("Cloud: width = %d, height = %d", msg->width, msg->height);

    // Extracting clusters.
    pcl::KdTree<VPoint>::Ptr tree (new pcl::KdTreeFLANN<VPoint>);
    tree->setInputCloud (msg);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<VPoint> ec;
    ec.setClusterTolerance(0.1); 
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(200);
    ec.setSearchMethod(tree);
    ec.setInputCloud(msg);
    ec.extract(cluster_indices);

    int numClusters = cluster_indices.size();
    printf("\tNum Clusters: %i\n", numClusters);

    BOOST_FOREACH(const pcl::PointIndices &clusterIndex, cluster_indices) {
      velodyne_pcl::PointXYZIR point;
      long ring = 0;
      point.x = point.y = point.z = point.intensity = point.ring = 0;
      for (unsigned int j = 0; j < clusterIndex.indices.size(); j++) {
        point.x += msg->points[clusterIndex.indices[j]].x;
        point.y += msg->points[clusterIndex.indices[j]].y;
        point.z += msg->points[clusterIndex.indices[j]].z;
        point.intensity += msg->points[clusterIndex.indices[j]].intensity;
        ring += msg->points[clusterIndex.indices[j]].ring;
      }
      point.intensity /= clusterIndex.indices.size();
      ring /= clusterIndex.indices.size();
      point.z /= clusterIndex.indices.size();
      point.x /= clusterIndex.indices.size();
      point.y /= clusterIndex.indices.size();
      
      printf ("\t\t(%f, %f, %f, %f, %li)\n", point.x, point.y, point.z, point.intensity, ring);
      
    }
  }

} // namespace velodyne_pcl

// Register this plugin with pluginlib. Names must match nodelets.xml.
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_pcl, ExampleSubscriberNodelet,
                        velodyne_pcl::ExampleSubscriberNodelet, nodelet::Nodelet);
