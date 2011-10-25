/* -*- mode: C++ -*- */
/*  Copyright (C) 2010 UT-Austin &  Austin Robot Technology,
 *  David Claridge, Michael Quinlan
 * 
 *  License: Modified BSD Software License 
 */


#ifndef _HEIGHT_MAP_H_
#define _HEIGHT_MAP_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <velodyne/data.h>

class HeightMap
{
public:

  /** Constructor
   *
   *  @param node NodeHandle of this instance
   *  @param private_nh private NodeHandle of this instance
   */
  HeightMap(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~HeightMap();

  /** callback to process data input
   *
   *  @param scan vector of input 3D data points
   *  @param stamp time stamp of data
   *  @param frame_id data frame of reference
   */
  void processData(const sensor_msgs::PointCloud::_points_type &scan,
                   const ros::Time &stamp, const std::string &frame_id);

private:
  void constructFullClouds(const sensor_msgs::PointCloud::_points_type &scan,
                           unsigned npoints, size_t &obs_count, size_t &empty_count);
  void constructGridClouds(const sensor_msgs::PointCloud::_points_type &scan,
                           unsigned npoints, size_t &obs_count, size_t &empty_count);


  // Parameters that define the grids and the height threshold
  // Can be set via the parameter server
  int grid_dim_;
  double m_per_cell_;
  double height_diff_threshold_;
  bool full_clouds_;

  // Point clouds generated in processData
  sensor_msgs::PointCloud obstacle_cloud_;            
  sensor_msgs::PointCloud clear_cloud_;            

  // Publishers
  ros::Publisher obstacle_publisher_;
  ros::Publisher clear_publisher_;
};

#endif
