/*
 *  Copyright (C) 2009 Jack O'Quin, Chau Nguyen
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node converts raw Velodyne HDL-64E 3D LIDAR data to a
    sensor_msgs/PointCloud.

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <velodyne/data.h>
#include <velodyne/running_stats.hh>

#include "Point.h"
#include "Cell.h"
#include "RadialGrid.h"

#include <iostream>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define NODE "hybrid_3d"

using namespace velodyne_common;

// command options
static int qDepth = 1;                  // ROS topic queue size

// local static data
static RunningStats latency;
static uint32_t droppedMsgs = 0;
static velodyne::DataXYZ *data = NULL;
static ros::Publisher output;

sensor_msgs::PointCloud pc;             // outgoing PointCloud message
//nav_msgs::OccupancyGrid grid; 		// radial occupancy grid
RadialGrid g;
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
  // set the exact point cloud size -- the vectors should already have
  // enough space
  size_t npoints = scan.size();
  pc.points.resize(npoints);

  // pass along original time stamp and frame of reference
  pc.header.stamp = hdr->stamp;
  pc.header.frame_id = hdr->frame_id; 

  double SPAN_ANGLE = 180;
  double MAX_DISTANCE = 60;
  double MIN_DISTANCE = 1.4;
  double MAX_HEIGHT = 2.5;
  bool FULL_PICTURE=false;
  for (unsigned i = 0; i < npoints; ++i)
  {
      double new_x = 0.438225 + 0.999278*scan[i].x + 0.0342887*scan[i].y + 0.016353*scan[i].z;
      double new_y = 0.250727 - 0.0341911*scan[i].x + 0.999396*scan[i].y - 0.0062125*scan[i].z;
      double new_z = 2.19473 - 0.0165561*scan[i].x + 0.00564889*scan[i].y + 0.999847*scan[i].z;
      double d = sqrt(new_x*new_x+new_y*new_y);
      if (d>=MIN_DISTANCE && d<=MAX_DISTANCE && new_z<=MAX_HEIGHT) {
	 double thetaInDegree = scan[i].heading*180/M_PI;
         if (thetaInDegree>=360) thetaInDegree -=360;
         if (thetaInDegree<0) thetaInDegree += 360;
         if (thetaInDegree<=SPAN_ANGLE || thetaInDegree>=360-SPAN_ANGLE){ 
            Point p(new_x,new_y,new_z,thetaInDegree);
            g.addPoint(p);
         }
      }
  }
  bool exist = g.finishAddingPoints();
  if (exist==true){
     int n=0;
     int rows = g.getGridRows();
     int cols = g.getGridCols();
     //int max = g.getSegmentNumber();
    // grid.info.width = cols;
    // grid.info.height = rows;
  //   ofstream outFile("segments.txt");
     int count = 0;
     Cell** c = g.getGrid();
     for(int i=0;i<rows;i++)
        for(int j=0;j<cols;j++)
        {
	   count++;
           int s = c[i][j].getSegNum();
       /*    if (s==0)
	      grid.data[count] = 0;
	   else if (s==max-1)
		grid.data[count] = 100;
           else grid.data[count] = -1;*/
	 //  if (s!=-1)
           if (s==0)
	   {
	      if (FULL_PICTURE==true){
              	vector<Point> v = c[i][j].getAllPoints();
              	for(int k=0;k<(int)v.size();k++){
                  Point pt = v.at(k);
		  pc.points[n+k].x = pt.getX();
      	          pc.points[n+k].y = pt.getY();
                  pc.points[n+k].z = pt.getZ();
	//	  outFile << s << " " << pt.getX() << " " << pt.getY() << " " << pt.getZ() << endl;
              	}
              	n +=(int) v.size();
	      }
	      else{
	      	Point pt = c[i][j].getPoint();
		pc.points[n].x = pt.getX();
		pc.points[n].y = pt.getX();
		pc.points[n].z = pt.getZ();
	//	outFile << s << " " << pt.getX() << " " << pt.getY() << " " << pt.getZ() << endl;
		n++;
	      }
           }
	}
	pc.points.resize(n);
  	output.publish(pc);
	//output.publish(grid);
//	outFile.close();
//        cout<<"finish publishing point cloud"<<endl;
  }
  //else cout<<"not enough data"<<endl;
  g.clear();
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

  output = node.advertise<sensor_msgs::PointCloud>("velodyne/hybridcloud",
                                                   qDepth);

  // preallocate the anticipated amount of space for the point cloud
  pc.points.resize(velodyne::SCANS_PER_REV);

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
