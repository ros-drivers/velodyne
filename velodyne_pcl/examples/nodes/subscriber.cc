#include <ros/ros.h>

#include <velodyne_pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <boost/foreach.hpp>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/tree_types.h>
#include <pcl/kdtree/impl/tree_types.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/impl/organized_data.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>

typedef velodyne_pcl::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

void callback(const VPointCloud::ConstPtr& msg) {
  ROS_INFO("Cloud: width = %d, height = %d", msg->width, msg->height);

  // Extracting clusters.
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<VPoint> ec;
  ec.setClusterTolerance(0.1); 
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(200);
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "velodyne_example_sub");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<VPointCloud>("velodyne/pointcloud", 1, callback);
  ros::spin();
}

