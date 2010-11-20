/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node captures Velodyne HDL-64E 3D LIDAR data and prints
    it in several human-readable file formats.

*/

#include <ros/ros.h>

#include <velodyne/data.h>

#define NODE "velodyne_print"

using namespace velodyne_common;

// command options
static int numPackets = 0;
static std::string ofile = "";
static int qDepth = 1;                  // ROS topic queue size

// local static data
static velodyne::Data *data = NULL;

/** \brief print Velodyne data points using data class */
void printPoints(size_t npackets)
{
  static int packetCount = 0;

  data->print();

  // see if this packet processed satisfied numPackets
  packetCount += npackets;
  if ((numPackets > 0) && (packetCount >= numPackets))
    {
      ROS_INFO("no more packets needed");
      data->shutdown();                 // shut down the data stream
    }
}

/** \brief callback for laser scans */
void processScans(const std::vector<velodyne::laserscan_t> &scan)
{
  printPoints(scan.size() / velodyne::SCANS_PER_PACKET);
}

/** \brief callback for XYZ points */
void processXYZ(const std::vector<velodyne::laserscan_xyz_t> &scan)
{
  printPoints(scan.size() / velodyne::SCANS_PER_PACKET);
}

void displayHelp() 
{
  std::cerr << "write Velodyne data in other formats\n"
            << std::endl
            << "Usage: rosrun velodyne_file print <options>\n"
            << std::endl
            << "Options:\n"
            << "\t -h, -?        print usage message\n"
            << "\t -t <type>     define output type \n"
          //<< "\t\t -t1   formatted dump of raw input\n"
            << "\t\t -t2   laser, direction, x, y, z, intensity (default) \n"
            << "\t\t -t3   theta,phi,distance,intensity\n"
          //<< "\t\t -t4   x,y,z,r,g,b (where r,g,b are all set to 255)\n"
            << "\t -n <integer>  read this many packets (default: all)\n"
            << "\t -q <integer>  set ROS topic queue depth (default: 1)\n"
            << "\t -f <filename> write output to this file (default: none)\n"
            << "\t -f-           write data to stdout\n"
            << std::endl
            << "Example:\n"
            << "  rosrun velodyne_file print -t2 -n260 -f onerev.vxyz\n"
            << std::endl;
}


/** get command line and ROS parameters
 *
 * \returns 0 if successful
 */
int getParameters(int argc, char *argv[])
{
  /// \todo use extension to set type
  static int type = 2;

  // use getopt to parse the flags
  char ch;
  const char* optflags = "f:n:q:t:h?";
  while(-1 != (ch = getopt(argc, argv, optflags)))
    {
      switch(ch)
        {
        case 'f':
          ofile = std::string(optarg);
          break;
        case 'n':
          numPackets = atoi(optarg);
          break;
        case 'q':
          qDepth = atoi(optarg);
          if (qDepth < 1)
            qDepth = 1;
          break;
        case 't':
          type = atoi(optarg);
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

  ROS_INFO("options: -n%d -t%d -f %s -q%d", numPackets, type,
           ofile.c_str(), qDepth);

  switch (type)
    {
    case 2:
      {
        velodyne::DataXYZ *dxyz = new velodyne::DataXYZ(ofile);
        dxyz->getParams();
        dxyz->subscribeXYZ(processXYZ);
        data = dxyz;                    // save pointer with base type
        break;
      }
    case 3:
      {
        velodyne::DataScans *dscans = new velodyne::DataScans(ofile);
        dscans->getParams();
        dscans->subscribeScans(processScans);
        data = dscans;                  // save pointer with base type
        break;
      }
    default:
      {
        ROS_FATAL("invalid -t option value: %d", type);
        displayHelp();
        return 1;
      }
    }

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
                   &velodyne::Data::processRawScan, data,
                   ros::TransportHints().tcpNoDelay(true));

  ros::spin();                          // handle incoming data

  data->shutdown();
  delete data;

  return 0;
}
