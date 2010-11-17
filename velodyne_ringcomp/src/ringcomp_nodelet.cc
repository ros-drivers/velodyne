/*
 *  Copyright (C) 2009 Austin Robot Technology, Joseph Campbell
 *  Copyright (C) 2010 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS nodelet processes raw Velodyne HDL-64E 3D LIDAR data to
    detect obstacles using ring compression.
*/

#include <cmath>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>

#include <velodyne/ring_sequence.h>

namespace ringcomp_nodelet
{
  class RingCompNodelet: public nodelet::Nodelet
  {
  public:

    RingCompNodelet() {}
    void onInit();

  private:

    void compInitialize();
    bool findChannels(const sensor_msgs::PointCloudPtr &scan);
    void printmatrix(geometry_msgs::Point32 mat[][velodyne::N_LASERS]);
    void processPointCloud(const sensor_msgs::PointCloudPtr &scan);
    int  radtodeg(float rad);
    void ringMeasure(sensor_msgs::PointCloud &pc,
                     geometry_msgs::Point32 vect[][velodyne::N_LASERS]);

    /** push a point onto a point cloud */
    inline void pushPoint(sensor_msgs::PointCloud &pc,
                          const geometry_msgs::Point32 &pt)
    {
      pc.points.push_back(pt);
    }

    /** Euclidean distance calculation */
    inline float twodDistance(const geometry_msgs::Point32 &p1,
                              const geometry_msgs::Point32 &p2)
    {
      float dx = p1.x - p2.x;
      float dy = p1.y - p2.y;
      return sqrt((dx*dx) + (dy*dy));
    }

    ros::Subscriber velodyne_scan_;
    ros::Publisher output_;


    /// configuration parameters
    typedef struct {
      double min_obstacle;             ///< minimum obstacle height
      double vel_height;               ///< Velodyne height above ground
    } Config;
    Config config_;

    int   ring_;                      ///< ring number of current scan
    int   ringChan_;                  ///< ring channel number
    float heading_;                   ///< heading of current scan
    int   headingChan_;               ///< heading channel number
  };

  const int N_ANGLES = 360;             ///< number of angles in ring matrix

  //contains laser angle with verticle by rank
  const float LAZ_ANG[velodyne::N_LASERS] = {
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
    92.02081230
  };

  //contains distance between rings on flat ground by laser rank
  const float FLAT_COMP[velodyne::N_LASERS-1] = {
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
  //that will signify an obstacle for case 1 based on min_obstacle
  float COMP_BASE[velodyne::N_LASERS-1];
  float CASE_1_RANGES[velodyne::N_LASERS];

  int CASE_1_LIM; //compInitialize sets to upper bound laser rank of case 1 lasers
  int CASE_3_LIM = 58; //first laser rank with pos angle

  /** nodelet initialization */
  void RingCompNodelet::onInit()
  {
    ros::NodeHandle private_nh = getPrivateNodeHandle();
    private_nh.param("min_obstacle", config_.min_obstacle, 0.15);
    NODELET_INFO_STREAM("minimum obstacle height: " << config_.min_obstacle);

    /// @todo figure out where to get device height from without
    ///       adding a vehicle-specific dependency
    private_nh.param("vel_height", config_.vel_height, 2.4);
    NODELET_INFO_STREAM("height of Velodyne above ground: " << config_.vel_height);

    // Subscribe to Velodyne input -- make sure queue depth is minimal,
    // so any missed scans are discarded.  Otherwise latency gets out of
    // hand.  It's bad enough anyway.
    ros::NodeHandle node = getNodeHandle();
    velodyne_scan_ = node.subscribe("velodyne/pointcloud", 1,
                                    &RingCompNodelet::processPointCloud, this,
                                    ros::TransportHints().tcpNoDelay(true));

    output_ = node.advertise<sensor_msgs::PointCloud>("velodyne/obstacles", 10);

    compInitialize();
  }

  /** initializes obstacle ranges for case 1 lasers and compressions for case 2 */
  void RingCompNodelet::compInitialize()
  {
  
    int i;
    for (i = 0; i < velodyne::N_LASERS; i++){
      COMP_BASE[i] =
        (FLAT_COMP[i]
         - (config_.min_obstacle * tan(LAZ_ANG[i+1] * (M_PI/180))));
    }
  
    i = 0;
    while(COMP_BASE[i] < 0 && i < velodyne::N_LASERS){
      i++;
    }
    CASE_1_LIM = i;

    i = 0;
    float tang;
    while (i < CASE_1_LIM){
      tang = tan(LAZ_ANG[i+1] * (M_PI/180));
      CASE_1_RANGES[i] = tang * (config_.vel_height - config_.min_obstacle);
      i++;
    }
  }

  /** Converts radian input to integer degrees between 0 and 359 */
  int RingCompNodelet::radtodeg(float rad)
  {
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
      deg = deg + N_ANGLES;
    
    return deg;
  }

  void RingCompNodelet::ringMeasure(sensor_msgs::PointCloud &pc,
                                    geometry_msgs::Point32 vect[][velodyne::N_LASERS])
  {
    for(int i = 0; i < N_ANGLES; i++)
      {
        for(int j = 0; j < CASE_3_LIM; j++)
          {
            if( j < CASE_1_LIM)
              {
                if( twodDistance(vect[i][j], vect[i][j+1]) < .03)
                  {
                    if (vect[i][j+1].x < CASE_1_RANGES[j])
                      {
                        pushPoint(pc, vect[i][j+1]);
                      }
                  }
              }
            else if(twodDistance(vect[i][j], vect[i][j+1]) < COMP_BASE[j])
              {
                pushPoint(pc, vect[i][j+1]);
              }
          }
      }
  }

  void RingCompNodelet::printmatrix(geometry_msgs::Point32 mat[][velodyne::N_LASERS])
  {
    for(int i = 0; i < N_ANGLES; i++)
      {
        for(int j = 0; j < velodyne::N_LASERS; j++)
          {
            std::cout << mat[i][j].x << " " << mat[i][j].y << " " << mat[i][j].z << '\n';
          }
      } 
  }

  /** find input channels
   *
   *  @returns true if successful
   *  @post Sets ringChan_, headingChan_ to agree with current scan.
   */
  bool RingCompNodelet::findChannels(const sensor_msgs::PointCloudPtr &scan)
  {
    ringChan_ = -1;
    headingChan_ = -1;

    // search the input channel names
    for (unsigned ch = 0; ch < scan->channels.size(); ++ch)
      {
        if (scan->channels[ch].name == "ring")
          ringChan_ = ch;
        else if (scan->channels[ch].name == "heading")
          headingChan_ = ch;
      }

    // successful if both ring and heading channels were found
    return (ringChan_ >= 0 && headingChan_ >= 0);
  }

  /** \brief callback for XYZ point cloud */
  void RingCompNodelet::processPointCloud(const sensor_msgs::PointCloudPtr &scan)
  {
    if (output_.getNumSubscribers() == 0)
      return;                           // nothing to do

    if (!findChannels(scan))
      {
        ROS_ERROR_THROTTLE(10, "ring compression requires heading and ring channels");
        return;
      }

    // allocate a new shared output pointer for zero-copy sharing with other nodelets
    sensor_msgs::PointCloudPtr outPtr(new sensor_msgs::PointCloud);
    outPtr->header.stamp = scan->header.stamp;
    outPtr->header.frame_id = scan->header.frame_id;
  
    // matrix of points to process
    geometry_msgs::Point32 data_mat[N_ANGLES][velodyne::N_LASERS];

    // for each input scan point
    for (unsigned i = 0; i < scan->points.size(); ++i)
      {
        // store points in the matrix
        geometry_msgs::Point32 pt = scan->points[i];
        ring_ = (int) rint(scan->channels[ringChan_].values[i]);
        heading_ = scan->channels[headingChan_].values[i];
	data_mat[radtodeg(heading_)][ring_] = pt;
      }

    // run the ring compression algorithm
    ringMeasure(*outPtr, data_mat);

    // publish the results
    output_.publish(outPtr);
  }

// Register this plugin with pluginlib.  Names must match ringcomp_nodelet.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_ringcomp, RingCompNodelet,
                        ringcomp_nodelet::RingCompNodelet,
                        nodelet::Nodelet);

}; // namespace ringcomp_nodelet
