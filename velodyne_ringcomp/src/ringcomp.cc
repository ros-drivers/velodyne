
/*
 *  Copyright (C) 2009 Austin Robot Technology, Joseph Campbell
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node processes raw Velodyne HDL-64E 3D LIDAR data using
    ring compression.

*/

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <velodyne/data.h>

#define NODE "ringcomp"

using namespace velodyne_common;
// command options
static int qDepth = 1;                  // ROS topic queue size

// local static data
static velodyne::DataXYZ *data = NULL;
static ros::Publisher output;

//constants for processXYZ (I know this is ugly)
velodyne::laserscan_xyz_t data_mat[360][64];


bool first_run = true;

//used to convert laser number to matrix index
std::map<int, int> MAP_ORDER;

//height of the velodyne in meteres
const int VEL_HEIGHT = 2.2;

//definition of an obstacle
float MIN_OBST = .15;
//used only to initialize MAP_ORDER
const int TEMP_LAZ_ORDER[64] = {6,7,10,11,0,1,4,5,8,9,14,15,18,19,22,
			  23,12,13,16,17,20,21,26,27,30,31,2,3,
			  24,25,28,29,38,39,42,43,32,33,36,37,40,
			  41,46,47,50,51,54,55,44,45,48,49,52,53,
			  58,59,62,63,34,35,56,57,60,61};

//contains laser angle with verticle by rank
const float LAZ_ANG[64] = {
  65.28896700,
  65.72367900,
  66.23703200,
  66.74982800,
  67.26211400,
  67.77392800,
  68.28531500,
  68.79631200,
  69.30696900,
  69.81731800,
  70.32740600,
  70.83727100,
  71.44216500,
  71.85649700,
  72.36593800,
  72.87531900,
  73.38468200,
  73.89406200,
  74.40350400,
  74.91304600,
  75.42272900,
  75.93259500,
  76.44268200,
  76.95303200,
  77.46368700,
  77.97468600,
  78.48607200,
  78.99788600,
  79.51017100,
  80.02296830,
  80.53632070,
  81.05027200,
  81.47918800,
  81.82011130,
  82.16085960,
  82.50144530,
  82.84188080,
  83.18217850,
  83.52234980,
  83.86240720,
  84.20236300,
  84.54222920,
  84.88201760,
  85.22174020,
  85.56140950,
  85.90103630,
  86.24063370,
  86.58021330,
  86.91978670,
  87.25936630,
  87.59896350,
  87.93859080,
  88.27825960,
  88.61798240,
  88.95777070,
  89.29763699,
  89.63759261,
  89.97765022,
  90.31782165,
  90.65811908,
  90.99855500,
  91.33914100,
  91.67988900,
  92.02081230};

//contains distance between rings on flat ground by laser rank
const float FLAT_COMP[63] = {
  0.09711589596,
  0.11897830436,
  0.12378418455,
  0.12892017590,
  0.13441452666,
  0.14030307990,
  0.14662184047,
  0.15341750055,
  0.16073317108,
  0.16862812847,
  0.17716167015,
  0.22232028841,
  0.16052564727,
  0.20735797309,
  0.21926581262,
  0.23228723239,
  0.24656171665,
  0.26226037120,
  0.27957481708,
  0.29873614251,
  0.32001696732,
  0.34373865361,
  0.37029194867,
  0.40014576254,
  0.43386762661,
  0.47215933933,
  0.51588306306,
  0.56611903612,
  0.62422533335,
  0.69193812767,
  0.77149948941,
  0.71448352613,
  0.62093965520,
  0.67420617492,
  0.73471590764,
  0.80384430580,
  0.88330959147,
  0.97527762014,
  1.08251992717,
  1.20861618705,
  1.35825959140,
  1.53770104317,
  1.75541534995,
  2.02312275695,
  2.35738526852,
  2.78227842586,
  3.33380543850,
  4.06783875008,
  5.07489914418,
  6.50953337943,
  8.65372736099,
  12.06773066628,
  18.00087834321,
  29.73996140445,
  58.52760804860,
  168.35270037068,
  5292.09791678684,
  -6036.51251234878,
  205.08091492193,
  65.30300971227,
  32.10944334797,
  19.09725472764,
  12.66324881604};

//after running rangeInitialize(), this list contains the max ground distance
//that will signify an obstacle for case 1 based on MIN_OBST
float COMP_BASE[63];

float CASE_1_RANGES[64];
const int NUM_LAZERS = 64;

int CASE_1_LIM; //compInitialize sets to upper bound laser rank of case 1 lasers
int CASE_3_LIM = 58; //first laser rank with pos angle

//helper functions for ringcomp

float twodDistance(velodyne::laserscan_xyz_t p1, velodyne::laserscan_xyz_t p2){
  
  float result;
  float dx;
  float dy;
  
  dx = p1.x - p2.x;
  dy = p1.y - p2.y;

  result = sqrt( (dx*dx) + (dy*dy));

  return result;
}

void mapInitialize(){
  
  for (int i = 0; i < NUM_LAZERS; i++){
    MAP_ORDER[ TEMP_LAZ_ORDER[i] ] = i;
  }

}

//initializes obstacle ranges for case 1 lasers and compressions for case 2
void compInitialize(){
  
  int i;
  for (i = 0; i < 64; i++){
    COMP_BASE[i] = FLAT_COMP[i] - (MIN_OBST * tan(LAZ_ANG[i+1] * (M_PI/180)));
  }

  
  i = 0;
  while(COMP_BASE[i] < 0 && i < 64){
    i++;
  }
  CASE_1_LIM = i;

  i = 0;
  float tang;
  while (i < CASE_1_LIM){
    tang = tan(LAZ_ANG[i+1] * (M_PI/180));
    CASE_1_RANGES[i] = tang * (VEL_HEIGHT - MIN_OBST);
    i++;
  }

}

//Converts radian input to degrees between 0 and 359
int radtodeg(float rad){
  
  //Convert radians to degrees
  rad = rad * (180 / M_PI);
  
  //Adding .5 will cause a round instead of a truncate
  if ( rad > 0 )
    rad = rad + 0.5;
  if ( rad < 0 )
    rad = rad - 0.5;
	
  //convert the float to an int
  int deg = static_cast<int>(rad);
  
  //Convert negative angles to pos for indexing into the matrix
  if (deg < 0)
    deg = deg + 360;
    
  return deg;

}

sensor_msgs::PointCloud pc;

void ringMeasure(velodyne::laserscan_xyz_t vect[][64]){
  int pcindex = 0;
#define DEBUG_TO_FILE 0
#if DEBUG_TO_FILE
  std::ofstream fout;
  fout.open("velodyne_ringcomp_output.txt");
#endif
  velodyne::laserscan_xyz_t p;
  pc.points.resize(100000);
  for(int i = 0; i < 360; i++){
    for(int j = 0; j < CASE_3_LIM; j++){
      if( j < CASE_1_LIM){
	if( twodDistance(vect[i][j], vect[i][j+1]) < .03){
	  if (vect[i][j+1].x < CASE_1_RANGES[j]){
	    p = vect[i][j+1];
	    pc.points[pcindex].x = p.x;
	    pc.points[pcindex].y = p.y;
            pc.points[pcindex].z = p.z;
	    ++pcindex;
#if DEBUG_TO_FILE
	    fout << p.x << ',' << p.y << ',' << p.z << ',';
	    fout << 255 << ',' << 0 << ',' << 0;
	    fout << '\n';
#endif	 
	  }
	}
      }
      else{
	if(twodDistance(vect[i][j], vect[i][j+1]) < COMP_BASE[j]){
	  p = vect[i][j+1];
	  pc.points[pcindex].x = p.x;
	  pc.points[pcindex].y = p.y;
          pc.points[pcindex].z = p.z;
	  ++pcindex;
#if DEBUG_TO_FILE
	  fout << p.x << ',' << p.y << ',' << p.z << ',';
	  fout << 255 << ',' << 0 << ',' << 0;
	  fout << '\n';
#endif	 
	}
      }
    }
  }
  pc.points.resize(pcindex);
  output.publish(pc);

}

void printmatrix(velodyne::laserscan_xyz_t mat[][64]){

  for(int i = 0; i < 360; i++){
    for(int j = 0; j < 64; j++){
	std::cout << mat[i][j].x << " " << mat[i][j].y << " " << mat[i][j].z << '\n';
    }
  } 

}
/** \brief callback for XYZ points
 *
 * performs ring compression on Velodyne data points
 */

void processXYZ(const std::vector<velodyne::laserscan_xyz_t> &scan)
{
  // pass along original time stamp and frame of reference
  data->getMsgHeaderFields(pc.header.stamp, pc.header.frame_id);

  if (first_run)
    {
	compInitialize();
	mapInitialize();
	first_run = false;
    }
  
  //holds one point waiting to be put into the matrix
  velodyne::laserscan_xyz_t p;
  for (unsigned i = 0; i < scan.size(); ++i)
    {
      // put points from the scan into the matrix
	p = scan[i];
	data_mat[radtodeg(p.heading)][ MAP_ORDER[p.laser_number] ] = p;	
	
	
    }
    ringMeasure(data_mat);
}

void displayHelp() 
{
  // fix this for help with your node...
  //std::cerr << "format raw Velodyne data and republish as a PointCloud\n"
  //          << std::endl
  //          << "Usage: rosrun velodyne_file cloud <options>\n"
  //          << std::endl
  //          << "Options:\n"
  //          << "\t -h, -?       print usage message\n"
  //          << "\t -q <integer> set ROS topic queue depth (default: 1)\n"
  //          << std::endl
  //          << "Example:\n"
  //          << "  rosrun velodyne_file cloud -q2\n"
  //          << std::endl;
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

  output = node.advertise<sensor_msgs::PointCloud>("velodyne/obstacles",
                                                   qDepth);

  ros::spin();                          // handle incoming data

  data->shutdown();
  delete data;
  return 0;
}
