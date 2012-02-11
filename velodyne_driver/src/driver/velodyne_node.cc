/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver node for the Velodyne HDL-64E 3D LIDAR
 */

#include <string>

#include <ros/ros.h>

#include <velodyne_driver/input.h>
#include <velodyne_msgs/VelodyneScan.h>

#define NODE "velodyne_read"

// file global data
static velodyne_driver::Input *input;

// command options
static int qDepth = 1;                  // ROS topic queue size

void displayHelp() 
{
  std::cerr << "read and publish Velodyne raw packets"
            << std::endl << std::endl
            << "Usage: rosrun velodyne_read read <options>"
            << std::endl << std::endl
            << "Options:" << std::endl
            << "\t -h, -?          print usage message\n"
            << "\t -q <integer>    set ROS topic queue depth (default: 1)\n"
            << "\t -f <input-file> set PCAP dump input file"
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
  std::string dump_file = "";
  const char* optflags = "f:hq:?";
  while(-1 != (ch = getopt(argc, argv, optflags)))
    {
      switch(ch)
        {
        case 'f':
          dump_file = std::string(optarg);
          break;
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

      ROS_INFO("Velodyne scan queue depth = %d", qDepth);
    }

  // use private node handle to get parameters
  ros::NodeHandle private_nh("~");
  private_nh.getParam("pcap", dump_file);

  if (dump_file != "")
    {
      ROS_INFO("reading data from file: %s", dump_file.c_str());
      input = new velodyne_driver::InputPCAP(dump_file);
    }
  else
    {
      ROS_INFO("reading data from socket");
      input = new velodyne_driver::InputSocket();
    }

  return 0;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  if (getParameters(argc, argv) != 0)
    return 1;

  // open Velodyne input device or file
  int rc = input->vopen();
  if(rc)
    {
      ROS_FATAL("Cannot open Velodyne input.");
      return 2;
    }

  ros::Publisher output =
    node.advertise<velodyne_msgs::VelodyneScan>("velodyne/packets", qDepth);

  int npackets = velodyne_msgs::VelodyneScan::PACKETS_PER_REVOLUTION;

  // Loop until shut down.
  while(ros::ok())
    {
      // Allocate a new shared pointer for zero-copy sharing with other nodelets.
      velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);
      scan->packets.resize(npackets);

      // Since the velodyne delivers data at a very high rate, keep
      // reading and publishing scans as fast as possible.
      for (int i = 0; i < npackets; ++i)
        {
          while (true)
            {
              // keep reading until full packet received
              int rc = input->getPacket(&scan->packets[i]);
              if (rc == 0)              // got a full packet?
                break;
              if (rc < 0)               // end of file reached?
                goto terminate;
            }
        }

      // publish message using time of last packet read
      ROS_DEBUG("Publishing a full Velodyne scan.");
      scan->header.stamp = ros::Time(scan->packets[npackets - 1].stamp);
      scan->header.frame_id = "/velodyne";

      if (output.getNumSubscribers() > 0) // anyone subscribed?
        {
          output.publish(scan);
        }
    }

 terminate:
  input->vclose();
  delete input;
  return 0;
}
