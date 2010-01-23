/*
 *  Copyright (C) 2009 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node converts raw Velodyne HDL-64E 3D LIDAR data to a
    PointCloud.

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <velodyne/data.h>
#include <velodyne/running_stats.hh>

#define NODE "velodyne_cloud"

using namespace velodyne_common;

// command options
static int qDepth = 1;                  // ROS topic queue size

// local static data
static RunningStats latency;
static uint32_t droppedMsgs = 0;
static velodyne::DataXYZ *data = NULL;
static ros::Publisher output;

sensor_msgs::PointCloud pc;             // outgoing PointCloud message

/** \brief accumulate and log message latency, dropped messages */
void checkLatency(const roslib::Header *hdr)
{
  static bool firstMsg = true;          // have we seen any yet?
  static uint32_t nextSeq = 0;          // next sequence number expected

  // accumulate latency mean and variance
  double timeDelay = ros::Time::now().toSec() - hdr->stamp.toSec();
  latency.addSample(timeDelay);
  ROS_DEBUG("message %u received, latency: %.3f seconds",
            hdr->seq, timeDelay);

  // check for lost messages
  if (firstMsg)
    {
      firstMsg = false;
    }
  else if (nextSeq < hdr->seq)
    {
      // If the publisher terminates and then restarts, the sequence
      // numbers apparently reset.  We don't count that as a dropped
      // message.
      uint32_t missed = hdr->seq - nextSeq;
      if (droppedMsgs == 0)
        // only log INFO message the first time
        ROS_INFO("%u Velodyne messages dropped", missed);
      else
        // log DEBUG message for any remaining losses
        ROS_DEBUG("%u more Velodyne messages dropped", missed);
      droppedMsgs += missed;
    }
  nextSeq = hdr->seq + 1;
}


/** \brief callback for XYZ points
 *
 * publishes Velodyne data points as a point cloud
 */
void processXYZ(const std::vector<velodyne::laserscan_xyz_t> &scan)
{
  const roslib::Header *hdr = data->getMsgHeader();
  checkLatency(hdr);

  // pass along original time stamp and frame ID
  pc.header.stamp = hdr->stamp;
  pc.header.frame_id = hdr->frame_id;

  // set the exact point cloud size -- the vectors should already have
  // enough space
  size_t npoints = scan.size();
  pc.points.resize(npoints);
  pc.channels[0].values.resize(npoints);

  for (unsigned i = 0; i < npoints; ++i)
    {
      pc.points[i].x = scan[i].x;
      pc.points[i].y = scan[i].y;
      pc.points[i].z = scan[i].z;
      pc.channels[0].values[i] = (float) scan[i].intensity;
    }

  ROS_DEBUG("Publishing %u Velodyne points.", npoints);
  output.publish(pc);
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
  data->subscribeXYZ(processXYZ);

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
    node.subscribe("velodyne/rawscan", qDepth,
                   &velodyne::Data::processRawScan, (velodyne::Data *) data,
                   ros::TransportHints().tcpNoDelay(true));

  output = node.advertise<sensor_msgs::PointCloud>("velodyne/pointcloud",
                                                   qDepth);

  // preallocate the anticipated amount of space for the point cloud
  pc.points.resize(velodyne::SCANS_PER_REV);
  pc.channels.resize(1);
  pc.channels[0].name = "intensity";
  pc.channels[0].values.resize(velodyne::SCANS_PER_REV);

  ROS_INFO(NODE ": starting main loop");

  ros::spin();                          // handle incoming data

  ROS_INFO(NODE ": exiting main loop");
  ROS_INFO("message latency: %.3f +/- %.3f",
           latency.getMean(), latency.getStandardDeviation());
  if (droppedMsgs)
    ROS_INFO("total of %u Velodyne messages dropped", droppedMsgs);
  else
    ROS_INFO("no Velodyne messages dropped");

  data->shutdown();
  delete data;

  return 0;
}
