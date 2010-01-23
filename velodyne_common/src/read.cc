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

#include <velodyne/input.h>
#include <velodyne_common/RawScan.h>

#define NODE "velodyne_read"

// file global data
static velodyne::Input *input;

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

      ROS_INFO("velodyne/rawscan queue depth = %d", qDepth);
    }

  // use private node handle to get parameters
  ros::NodeHandle private_nh("~");
  private_nh.getParam("pcap", dump_file);

  using namespace velodyne;
  if (dump_file != "")
    {
      ROS_INFO("reading data from file: %s", dump_file.c_str());
      input = new InputPCAP(dump_file);
    }
  else
    {
      ROS_INFO("reading data from socket");
      input = new InputSocket();
    }

  return input->getParams();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  if (getParameters(argc, argv) != 0)
    return 1;

  using namespace velodyne_common;

  // open Velodyne input device or file
  int rc = input->vopen();
  if(rc)
    {
      ROS_FATAL("Cannot open Velodyne input.");
      return 2;
    }

  ros::Publisher output = node.advertise<RawScan>("velodyne/rawscan", qDepth);

  ROS_INFO(NODE ": starting main loop");

  RawScan rawscan;
  int npackets = RawScan::PACKETS_PER_REVOLUTION;
  rawscan.data.resize(npackets * RawScan::PACKET_SIZE);
  rawscan.header.frame_id = "/velodyne";

  // Loop until shut down.
  while(ros::ok())
    {
      double time;

      // Since the velodyne delivers data at a very high rate, keep
      // reading and publishing scans as fast as possible.
      int packets_left = input->getPackets(&rawscan.data[0], npackets, &time);

      if (packets_left != 0)            // incomplete scan received?
        {
          if (packets_left < 0)         // end of file reached?
            break;

          if (packets_left < npackets)  // partial scan read?
            ROS_WARN("Incomplete Velodyne scan: %d packets missing",
                     packets_left);
        }
      else
        {
          ROS_DEBUG("Publishing a full Velodyne scan.");
          rawscan.header.stamp = ros::Time(time);
          output.publish(rawscan);
        }
    }

  ROS_INFO(NODE ": exiting main loop");

  input->vclose();

  delete input;

  return 0;
}
