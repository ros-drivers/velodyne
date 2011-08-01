/*
 *  Copyright (C) 2009 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node converts raw Velodyne HDL-64E 3D LIDAR data to a
    PointCloud2.

*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne/data.h>
#include <velodyne/ring_sequence.h>

#include <velodyne_pcl/point_types.h>

#include <pcl/io/pcd_io.h>
//#include <pcl_ros/transforms.h>

#define NODE "velodyne_cloud2"

// command options
static int qDepth = 1;                  // ROS topic queue size

// local static data
static velodyne::DataXYZ *data = NULL;
static ros::Publisher output;

pcl::PointCloud<velodyne_pcl::PointXYZIR>::Ptr cloud;
sensor_msgs::PointCloud2 pc2_;         // outgoing PointCloud2 message


/** \brief callback for XYZ points
 *
 *  publishes Velodyne data points as a PointCloud2 message
 */
void processXYZ(const std::vector<velodyne::laserscan_xyz_t> &scan)
{
  // set the exact point cloud size
  size_t npoints = scan.size();
  cloud->points.resize(npoints);

  for (unsigned i = 0; i < npoints; ++i)
  {
    cloud->points[i].x = scan[i].x;
    cloud->points[i].y = scan[i].y;
    cloud->points[i].z = scan[i].z;
    cloud->points[i].intensity = (float) scan[i].intensity;
    cloud->points[i].ring = velodyne::LASER_RING[scan[i].laser_number];
  }

  // copy data to a ROS PointCloud2 message
  sensor_msgs::PointCloud2 pc2;
  pcl::toROSMsg(*cloud, pc2);
  
  // pass along original time stamp and frame ID
  data->getMsgHeaderFields(pc2.header.stamp, pc2.header.frame_id);

#if 0  // do not transform, republish entire scan in original frame ID
  tf::TransformListener tfListener;
  
  const char* availibleFrames = tfListener.allFramesAsString().c_str();
  
  if(!tfListener.frameExists(pc.header.frame_id))
    {
      ROS_INFO("Cannot find frame %s. The known existing frames are %s.",
               pc.header.frame_id.c_str(), availibleFrames);
    }
  
  std::string target_frame = "odom";
  
  try
    {
      pcl_ros::transformPointCloud(target_frame, pc, pc_out, tfListener);
    }
  catch (tf::TransformException ex)
    {
      // only log tf error once every 20 times
      ROS_WARN_THROTTLE(20, "%s", ex.what());
      return;                           // skip this packet
    }  
#endif
        
  ROS_DEBUG_STREAM("Publishing " << pc2.width * pc2.height
                   << " Velodyne points in frame " << pc2.header.frame_id);

  ROS_DEBUG_STREAM("Publishing " << npoints << " Velodyne points.");
  output.publish(pc2);
}

void displayHelp() 
{
  std::cerr << "format raw Velodyne data and republish as a PointCloud\n"
            << std::endl
            << "Usage: rosrun velodyne_file cloud <options>\n"
            << std::endl
            << "Options:\n"
            << "\t -h, -?       print usage message\n"
            << "\t -q <integer> set ROS topic queue depth (default: 1)\n"
            << std::endl
            << "Example:\n"
            << "  rosrun velodyne_file cloud -q2\n"
            << std::endl;
}


/** get command line and ROS parameters
 *
 * \returns 0 if successful
 */
int getParameters(int argc, char *argv[])
{
  // use getopt to parse the flags
  char ch;
  const char* optflags = "hq:?";
  while(-1 != (ch = getopt(argc, argv, optflags)))
    {
      switch(ch)
        {
        case 'q':
          qDepth = atoi(optarg);
          if (qDepth < 1)
            qDepth = 1;
          break;
        default:                        // unknown
          ROS_WARN("unknown parameter: %c", ch);
          // fall through to display help...
        case 'h':                       // help
        case '?':
          displayHelp();
          return 1;
        }
    }

  ROS_INFO("topic queue depth = %d", qDepth);

  data = new velodyne::DataXYZ();
  data->getParams();

  return 0;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  if (0 != getParameters(argc, argv))
    return 9;

  if (0 != data->setup())
    return 2;

  // subscribe to velodyne input -- make sure queue depth is minimal,
  // so any missed scans are discarded.  Otherwise latency gets out of
  // hand.  It's bad enough anyway.
  ros::Subscriber velodyne_scan =
    data->subscribe(node, "velodyne/rawscan", qDepth,
                    boost::bind(&processXYZ, _1),
                    ros::TransportHints().tcpNoDelay(true));

  output = node.advertise<sensor_msgs::PointCloud2>("velodyne/pointcloud2",
                                                   qDepth);

  // preallocate space for PointXYZI data
  cloud.reset(new pcl::PointCloud<velodyne_pcl::PointXYZIR>);

  ROS_DEBUG(NODE ": starting main loop");

  ros::spin();                          // handle incoming data

  ROS_DEBUG(NODE ": exiting main loop");

  data->shutdown();
  delete data;

  return 0;
}
