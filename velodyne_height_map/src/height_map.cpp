/*  Copyright (C) 2010 UT-Austin &  Austin Robot Technology,
 *  David Claridge, Michael Quinlan
 * 
 *  License: Modified BSD Software License 
 */

#include <velodyne_height_map/height_map.h>

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))

HeightMap::HeightMap(ros::NodeHandle node, ros::NodeHandle nh)
{
  // Set parameter defaults
  grid_dim_ = 320;
  m_per_cell_ = 0.5;
  height_diff_threshold_ = 0.25;
  full_clouds_ = false;
  
  // Get a node handle with /velodyne_height_map prefix
  // Load parameters changes (if any)
  nh.getParam("grid_dim", grid_dim_);
  nh.getParam("m_per_cell", m_per_cell_);
  nh.getParam("height_threshold", height_diff_threshold_);  
  nh.getParam("full_clouds", full_clouds_);
  
  ROS_INFO("velodyne_height_map parameters: %i , %f, %f, %i",
           grid_dim_,m_per_cell_,height_diff_threshold_,full_clouds_);

  // preallocate the anticipated amount of space for the point clouds
  obstacle_cloud_.points.resize(velodyne::SCANS_PER_REV);
  //obstacle_cloud_.channels.resize(1);
  //obstacle_cloud_.channels[0].name = "intensity";
  //obstacle_cloud_.channels[0].values.resize(velodyne::SCANS_PER_REV);

  clear_cloud_.points.resize(velodyne::SCANS_PER_REV);
  //clear_cloud_.channels.resize(1);
  //clear_cloud_.channels[0].name = "intensity";
  //clear_cloud_.channels[0].values.resize(velodyne::SCANS_PER_REV);
  
  // Set up publishers  
  obstacle_publisher_ =
    node.advertise<sensor_msgs::PointCloud>("velodyne/obstacles",1);
  clear_publisher_ =
    node.advertise<sensor_msgs::PointCloud>("velodyne/clear",1);  
}

HeightMap::~HeightMap()
{
}

void HeightMap::constructFullClouds(const sensor_msgs::PointCloud::_points_type &scan,
                                    unsigned npoints, size_t &obs_count, size_t &empty_count)
{
  float min[grid_dim_][grid_dim_];
  float max[grid_dim_][grid_dim_];
  bool init[grid_dim_][grid_dim_];
  memset(&init, 0, grid_dim_*grid_dim_);
  
  // build height map
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
      if (!init[x][y]) {
        min[x][y] = scan[i].z;
        max[x][y] = scan[i].z;
        init[x][y] = true;
      } else {
        min[x][y] = MIN(min[x][y], scan[i].z);
        max[x][y] = MAX(max[x][y], scan[i].z);
      }
    }
  }

  // display points where map has height-difference > threshold
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {
      if ((max[x][y] - min[x][y] > height_diff_threshold_) ) {   
        obstacle_cloud_.points[obs_count].x = scan[i].x;
        obstacle_cloud_.points[obs_count].y = scan[i].y;
        obstacle_cloud_.points[obs_count].z = scan[i].z;
        //obstacle_cloud_.channels[0].values[obs_count] = (float) scan[i].intensity;
        obs_count++;
      } else {
        clear_cloud_.points[empty_count].x = scan[i].x;
        clear_cloud_.points[empty_count].y = scan[i].y;
        clear_cloud_.points[empty_count].z = scan[i].z;
        //clear_cloud_.channels[0].values[empty_count] = (float) scan[i].intensity;
        empty_count++;
      }
    }
  }
}

void HeightMap::constructGridClouds(const sensor_msgs::PointCloud::_points_type &scan,
                                    unsigned npoints, size_t &obs_count, size_t &empty_count)
{
  float min[grid_dim_][grid_dim_];
  float max[grid_dim_][grid_dim_];
  float num_obs[grid_dim_][grid_dim_];
  float num_clear[grid_dim_][grid_dim_];
  bool init[grid_dim_][grid_dim_];

  //memset(&init, 0, grid_dim_*grid_dim_);
  
  for (int x = 0; x < grid_dim_; x++) {
    for (int y = 0; y < grid_dim_; y++) {
      init[x][y]=false;
      num_obs[x][y]=0;
      num_clear[x][y]=0;
    }
  }

  // build height map
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
      if (!init[x][y]) {
        min[x][y] = scan[i].z;
        max[x][y] = scan[i].z;
        num_obs[x][y] = 0;
        num_clear[x][y] = 0;
        init[x][y] = true;
      } else {
        min[x][y] = MIN(min[x][y], scan[i].z);
        max[x][y] = MAX(max[x][y], scan[i].z);
      }
    }
  }

  // calculate number of obstacles in each cell
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {
      if ((max[x][y] - min[x][y] > height_diff_threshold_) ) {  
        num_obs[x][y]++;
      } else {
        num_clear[x][y]++;
      }
    }
  }

  // create clouds from grid
  double grid_offset=grid_dim_/2.0*m_per_cell_;
  for (int x = 0; x < grid_dim_; x++) {
    for (int y = 0; y < grid_dim_; y++) {
      if (num_obs[x][y]>0) {

        obstacle_cloud_.points[obs_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
        obstacle_cloud_.points[obs_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
        obstacle_cloud_.points[obs_count].z = height_diff_threshold_;
        //obstacle_cloud_.channels[0].values[obs_count] = (float) 255.0;
        obs_count++;
      }
      if (num_clear[x][y]>0) {
        clear_cloud_.points[empty_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
        clear_cloud_.points[empty_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
        clear_cloud_.points[empty_count].z = height_diff_threshold_;
        //clear_cloud_.channels[0].values[empty_count] = (float) 255.0;
        empty_count++;
      }
    }
  }
}

void HeightMap::processData(const sensor_msgs::PointCloud::_points_type &scan,
                            const ros::Time &stamp,
                            const std::string &frame_id)
{
  if ((obstacle_publisher_.getNumSubscribers() == 0)
      && (clear_publisher_.getNumSubscribers() == 0))
    return;
  
  // pass along original time stamp and frame ID
  obstacle_cloud_.header.stamp = stamp;
  obstacle_cloud_.header.frame_id = frame_id;

  // pass along original time stamp and frame ID
  clear_cloud_.header.stamp = stamp;
  clear_cloud_.header.frame_id = frame_id;

  // set the exact point cloud size -- the vectors should already have
  // enough space
  size_t npoints = scan.size();
  obstacle_cloud_.points.resize(npoints);
  //obstacle_cloud_.channels[0].values.resize(npoints);

  clear_cloud_.points.resize(npoints);
  //clear_cloud_.channels[0].values.resize(npoints);

  size_t obs_count=0;
  size_t empty_count=0;
  // either return full point cloud or a discretized version
  if (full_clouds_)
    constructFullClouds(scan,npoints,obs_count, empty_count);
  else
    constructGridClouds(scan,npoints,obs_count, empty_count);
  
  obstacle_cloud_.points.resize(obs_count);
  //obstacle_cloud_.channels[0].values.resize(obs_count);

  clear_cloud_.points.resize(empty_count);
  //clear_cloud_.channels[0].values.resize(empty_count);
  
  if (obstacle_publisher_.getNumSubscribers() > 0)
    obstacle_publisher_.publish(obstacle_cloud_);

  if (clear_publisher_.getNumSubscribers() > 0)
    clear_publisher_.publish(clear_cloud_);
}


