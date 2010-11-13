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
 *  a vector of individual laser scans.
 *
 *  \author Yaxin Liu
 *  \author Patrick Beeson
 *  \author Jack O'Quin
 */

#ifndef __VELODYNE_DATA_SCAN_H
#define __VELODYNE_DATA_SCAN_H

#include <velodyne/data_base.h>

namespace Velodyne
{
  /** \brief A single laser scan in Velodyne's frame of reference.
   *
   *   pitch is relative to the plane of the unit (in its frame of
   *   reference): positive is above, negative is below
   *
   *   heading is relative to the front of the unit (the outlet is the back):
   *   positive is clockwise  (yaw)
   */
  typedef struct laserscan
  {
    float range;                        ///< in meters
    float heading;                      ///< in radians
    float pitch;                        ///< in radians
    uint16_t revolution;
    uint8_t  laser_number;
    uint8_t  intensity;
  } laserscan_t;

  /** \brief type of callback function to receive laser scans
   *
   *  \param scan vector containing the laser scans for a single packet
   *  \param time ROS time stamp of packet
   *  \param frame_id ROS frame ID of packet
   */
  typedef boost::function<void(const std::vector<laserscan_t> &scan,
                               ros::Time time,
                               const std::string &frame_id)> scanCallback;

  /** \brief Convert Velodyne raw input to laserscans format */
  class DataScans: public Data
  {
  public:

    DataScans(std::string ofile="", std::string anglesFile="");
    virtual int print(void);
    virtual void processPacket(const velodyne_msgs::VelodynePacket *pkt,
                               const std::string &frame_id);

    /** \brief Subscribe to laser scans.
     *
     * \param node the ros::NodeHandle to use to subscribe.
     * \param base_topic the topic to subscribe to.
     * \param queue_size the subscription queue size
     * \param callback function or method to receive scan data
     * \param transport_hints optional transport hints for this subscription
     */
    ros::Subscriber subscribe(ros::NodeHandle node,
                              const std::string &topic,
                              uint32_t queue_size,
                              const scanCallback callback,
                              const ros::TransportHints &transport_hints
                                      = ros::TransportHints())
    {
      ROS_INFO("generic scan callback defined");
      cb_ = callback;
      return node.subscribe(topic, queue_size,
                            &Data::processScan, (Data *) this,
                            transport_hints);
    }

  protected:
    void packet2scans(const raw_packet_t *raw, laserscan_t *scans);

    // derived class data
    std::vector<laserscan_t> scans_;

  private:
    scanCallback cb_;                   ///< scan data callback
  };

} // velodyne namespace

#endif // __VELODYNE_DATA_SCAN_H
