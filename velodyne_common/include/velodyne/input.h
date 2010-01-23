/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Velodyne HDL-64E 3D LIDAR data input classes
 *
 *  \ingroup velodyne
 *
 *    These classes provide raw Velodyne LIDAR input packets from
 *    either a live socket interface or a previously-saved PCAP dump
 *    file.
 *
 *  Classes:
 *
 *     velodyne::Input -- virtual base class than can be used to access
 *                      the data independently of its source
 *
 *     velodyne::InputSocket -- derived class reads live data from the
 *                      device via a UDP socket
 *
 *     velodyne::InputPCAP -- derived class provides a similar interface
 *                      from a PCAP dump
 */

#ifndef __VELODYNE_INPUT_H
#define __VELODYNE_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>

#include <ros/ros.h>
#include <velodyne_common/RawScan.h>

namespace velodyne
{
  static uint16_t UDP_PORT_NUMBER = 2368;

  // Base Velodyne input class -- not used directly
  class Input
  {
  public:
    Input() {}
    ~Input() {}

    // Read velodyne packets.
    //
    // buffer = array to receive raw data packets
    // npacks = number of packets to read
    // data_time -> average time when data received
    //
    // returns: number of packets not read, if any
    //          0 if successful,
    //          -1 if end of file
    //
    virtual int getPackets(uint8_t *buffer, int npacks, double *data_time)
    {return 0;}

    /** \brief Get ROS parameters.
     *
     *  ROS parameter settings override constructor options
     *
     * \returns: 0, if successful;
     *           errno value, for failure
     */
    virtual int getParams(void)
    {return 0;}

    // Close the data socket or file
    //
    // returns: 0, if successful
    //          errno value, for failure
    //
    virtual int vclose(void)
    {return 0;}

    /** Open the data socket or file
     *
     * \returns: 0, if successful;
     *           -1 for failure
     * \todo return errno value for failure
     */
    virtual int vopen(void)
    {return 0;}
  };

  // live Velodyne input from socket.
  class InputSocket: public Input
  {
  public:
    InputSocket(uint16_t udp_port = UDP_PORT_NUMBER):
    Input()
    {
      udp_port_ = udp_port;
      sockfd_ = -1;
    }
    ~InputSocket() {}

    virtual int getPackets(uint8_t *buffer, int npacks, double *data_time);
    virtual int vclose(void);
    virtual int vopen(void);

  private:
    uint16_t udp_port_;
    int sockfd_;
  };


  /** Velodyne input from PCAP dump file.
   *
   * Dump files can be grabbed by libpcap, Velodyne's DSR software,
   * ethereal, wireshark, tcpdump, or the \ref vdump_command.
   */
  class InputPCAP: public Input
  {
  public:
    InputPCAP(std::string filename="",
              bool read_once=false,
              bool read_fast=false,
              double repeat_delay=0.0): Input()
    {
      filename_ = filename;
      fp_ = NULL;  
      pcap_ = NULL;  
      empty_ = true;
      read_once_ = read_once;
      if (read_once_)
        ROS_INFO("Read input file only once.");
      read_fast_ = read_fast;
      if (read_fast_)
        ROS_INFO("Read input file as quickly as possible.");
      repeat_delay_ = repeat_delay;
      if (repeat_delay_ > 0.0)
        ROS_INFO("Delay %.3f seconds before repeating input file.",
                 repeat_delay_);
    }
    ~InputPCAP() {}

    virtual int getPackets(uint8_t *buffer, int npacks, double *data_time);
    virtual int getParams(void);
    virtual int vclose(void);
    virtual int vopen(void);

  private:
    std::string filename_;
    FILE *fp_;
    pcap_t *pcap_;
    char errbuf_[PCAP_ERRBUF_SIZE];
    bool empty_;
    bool read_once_;
    bool read_fast_;
    double repeat_delay_;
  };

} // velodyne namespace

#endif // __VELODYNE_INPUT_H
