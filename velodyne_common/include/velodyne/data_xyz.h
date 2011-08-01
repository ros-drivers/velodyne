/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Velodyne HDL-64E 3D LIDAR data accessor
 *
 *  Derived class for unpacking raw Velodyne LIDAR packets into
 *  a vector of individual XYZ laser scan points.
 *
 *  \author Yaxin Liu
 *  \author Patrick Beeson
 *  \author Jack O'Quin
 */

#ifndef __VELODYNE_DATA_XYZ_H
#define __VELODYNE_DATA_XYZ_H

#include <velodyne/data_scans.h>

namespace Velodyne
{
  /** \brief a single laser scan in XYZ format
   *
   *  Data are in the Velodyne's frame of reference.
   */
  typedef struct laserscan_xyz
  {
    float x, y, z;
    float heading;
    uint16_t revolution;
    uint8_t  laser_number;
    uint8_t  intensity;
  } laserscan_xyz_t;

  /** \brief vector of XYZ laser scans */
  typedef std::vector<laserscan_xyz_t> xyz_scans_t;

  /** \brief type of callback function to receive XYZ laser scans
   *
   *  \param scan vector containing the XYZ scans for a single packet
   *  \param stamp ROS time stamp of packet
   *  \param frame_id ROS frame ID of packet
   */
  typedef boost::function<void(const xyz_scans_t &scan,
                               ros::Time stamp,
                               const std::string &frame_id)> xyzCallback;

  /** \brief Convert Velodyne raw input to XYZ format */
  class DataXYZ: public DataScans
  {
  public:

    DataXYZ(std::string ofile="", std::string anglesFile="");
    virtual int print(void);
    virtual void processPacket(const velodyne_msgs::VelodynePacket *pkt,
                               const std::string &frame_id);

    /** \brief Subscribe to XYZ laser scans.
     *
     * \param node the ros::NodeHandle to use to subscribe.
     * \param base_topic the topic to subscribe to.
     * \param queue_size the subscription queue size
     * \param callback function or method to receive XYZ data
     * \param transport_hints optional transport hints for this subscription
     */
    ros::Subscriber subscribe(ros::NodeHandle node,
                              const std::string &topic,
                              uint32_t queue_size,
                              const xyzCallback callback,
                              const ros::TransportHints &transport_hints
                                      = ros::TransportHints())
    {
      ROS_INFO("XYZ callback subscribed");
      cb_ = callback;
      return node.subscribe(topic, queue_size,
                            &Data::processScan, (Data *) this,
                            transport_hints);
    }

  protected:
    void scan2xyz(const laserscan_t *scan, laserscan_xyz_t *point);
    xyz_scans_t xyzScans_;              ///< vector of XYZ scans

  private:
    xyzCallback cb_;                    ///< XYZ packet callback
  };


} // velodyne namespace

#endif // __VELODYNE_DATA_XYZ_H
