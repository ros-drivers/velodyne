// Copyright 2007, 2009, 2010, 2015, 2019 Austin Robot Technology, Patrick Beeson, Jack O'Quin, AutonomouStuff  // NOLINT
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** \file
 *
 *  Input classes for the Velodyne HDL-64E 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#include "velodyne_driver/input.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include <velodyne_msgs/msg/velodyne_packet.hpp>

#include "velodyne_driver/time_conversion.hpp"

namespace velodyne_driver
{

static const size_t packet_size =
  sizeof(velodyne_msgs::msg::VelodynePacket().data);

////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS node handle for calling node.
 *  @param devip Device IP address.
 *  @param port UDP port number.
 */
Input::Input(rclcpp::Node * private_nh, const std::string & devip, uint16_t port)
: private_nh_(private_nh),
  devip_str_(devip),
  port_(port)
{
  if (!devip_str_.empty()) {
    RCLCPP_INFO(
      private_nh->get_logger(),
      "Only accepting packets from IP address: %s", devip_str_.c_str());
  }
}

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param devip Device IP address.
 *  @param port UDP port number.
 */
InputSocket::InputSocket(
  rclcpp::Node * private_nh,
  const std::string & devip,
  uint16_t port, bool gps_time)
: Input(private_nh, devip, port), gps_time_(gps_time)
{
  sockfd_ = -1;

  if (!devip_str_.empty()) {
    ::inet_aton(devip_str_.c_str(), &devip_);
  }

  // connect to Velodyne UDP port
  RCLCPP_INFO(private_nh->get_logger(), "Opening UDP socket: port %u", port);

  sockfd_ = ::socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1) {
    RCLCPP_ERROR(private_nh->get_logger(), "Error opening socket: %s", ::strerror(errno));
    return;
  }

  sockaddr_in my_addr{};                   // my address information
  my_addr.sin_family = AF_INET;            // host byte order
  my_addr.sin_port = htons(port);          // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

  // compatibility with Spot Core EAP, reuse port 2368
  int val = 1;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)) == -1) {
    RCLCPP_ERROR(private_nh->get_logger(), "Error setting REUSEADDR: %s", ::strerror(errno));
    return;
  }

  if (::bind(sockfd_, reinterpret_cast<sockaddr *>(&my_addr), sizeof(sockaddr)) == -1) {
    RCLCPP_ERROR(private_nh->get_logger(), "Error binding to socket: %s", ::strerror(errno));
    return;
  }

  if (::fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    RCLCPP_ERROR(
      private_nh->get_logger(),
      "Error setting socket to non-blocking: %s", ::strerror(errno));
    return;
  }

  RCLCPP_DEBUG(private_nh->get_logger(), "Velodyne socket fd is %d\n", sockfd_);
}

/** @brief destructor */
InputSocket::~InputSocket()
{
  (void) ::close(sockfd_);
}

/** @brief Get one velodyne packet. */
int InputSocket::getPacket(velodyne_msgs::msg::VelodynePacket * pkt, const double time_offset)
{
  rclcpp::Time time1;

  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 1000;  // one second (in msec)

  sockaddr_in sender_address{};
  socklen_t sender_address_len = sizeof(sender_address);

  while (true) {
    // Unfortunately, the Linux kernel recvfrom() implementation
    // uses a non-interruptible sleep() when waiting for data,
    // which would cause this method to hang if the device is not
    // providing data.  We poll() the device first to make sure
    // the recvfrom() will not block.
    //
    // Note, however, that there is a known Linux kernel bug:
    //
    //   Under Linux, select() may report a socket file descriptor
    //   as "ready for reading", while nevertheless a subsequent
    //   read blocks.  This could for example happen when data has
    //   arrived but upon examination has wrong checksum and is
    //   discarded.  There may be other circumstances in which a
    //   file descriptor is spuriously reported as ready.  Thus it
    //   may be safer to use O_NONBLOCK on sockets that should not
    //   block.

    // poll() until input available
    do {
      int retval = ::poll(fds, 1, POLL_TIMEOUT);
      if (retval < 0) {           // poll() error?
        if (errno != EINTR) {
          RCLCPP_ERROR(private_nh_->get_logger(), "poll() error: %s", ::strerror(errno));
        }

        return -1;
      }

      if (retval == 0) {          // poll() timeout?
        RCLCPP_WARN(private_nh_->get_logger(), "Velodyne poll() timeout");
        return -1;
      }

      if ((fds[0].revents & POLLERR) ||
        (fds[0].revents & POLLHUP) ||
        (fds[0].revents & POLLNVAL))  // device error?
      {
        RCLCPP_ERROR(private_nh_->get_logger(), "poll() reports Velodyne error");
        return -1;
      }
    } while ((fds[0].revents & POLLIN) == 0);

    time1 = private_nh_->get_clock()->now();

    // Receive packets that should now be available from the
    // socket using a blocking read.
    ssize_t nbytes = ::recvfrom(
      sockfd_, &pkt->data[0],
      packet_size, 0,
      reinterpret_cast<sockaddr *>(&sender_address),
      &sender_address_len);

    if (nbytes < 0) {
      if (errno != EWOULDBLOCK) {
        RCLCPP_ERROR(private_nh_->get_logger(), "recvfail: %s", ::strerror(errno));
        return -1;
      }
    } else if (static_cast<size_t>(nbytes) == packet_size) {
      // read successful,
      // if packet is not from the lidar scanner we selected by IP,
      // continue otherwise we are done
      if (!devip_str_.empty() &&
        sender_address.sin_addr.s_addr != devip_.s_addr)
      {
        continue;
      }
      break;  // done
    }

    RCLCPP_DEBUG(
      private_nh_->get_logger(),
      "incomplete Velodyne packet read: %zd bytes", nbytes);
  }

  rclcpp::Time time2 = private_nh_->get_clock()->now();
  if (!gps_time_) {
    // Average the times at which we begin and end reading.  Use that to
    // estimate when the scan occurred. Add the time offset.
    pkt->stamp = rclcpp::Time((time2.nanoseconds() + time1.nanoseconds()) / 2.0 + time_offset);
  } else {
    // time for each packet is a 4 byte uint located starting at offset 1200 in
    // the data packet
    // TODO(clalancette): What if the packet is shorter than 1204 bytes?
    pkt->stamp = rosTimeFromGpsTimestamp(time2, &(pkt->data[1200]));
  }

  return 0;
}

////////////////////////////////////////////////////////////////
// InputPCAP class implementation
////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param devip Device IP address.
 *  @param port UDP port number.
 *  @param packet_rate expected device packet frequency (Hz).
 *  @param filename PCAP dump file name.
 *  @param read_once Read the input file only once.
 *  @param read_fast Output the data from the input file as fast as possible.
 *  @param repeat_delay Seconds to wait between repeating input file.
 */
InputPCAP::InputPCAP(
  rclcpp::Node * private_nh, const std::string & devip, uint16_t port,
  double packet_rate, const std::string & filename, bool read_once,
  bool read_fast, double repeat_delay)
: Input(private_nh, devip, port),
  packet_rate_(packet_rate),
  filename_(filename),
  pcap_(nullptr),
  read_once_(read_once),
  read_fast_(read_fast),
  repeat_delay_(repeat_delay)
{
  empty_ = true;

  if (read_once_) {
    RCLCPP_INFO(private_nh->get_logger(), "Read input file only once.");
  }
  if (read_fast_) {
    RCLCPP_INFO(private_nh->get_logger(), "Read input file as quickly as possible.");
  }
  if (repeat_delay_ > 0.0) {
    RCLCPP_INFO(
      private_nh->get_logger(), "Delay %.3f seconds before repeating input file.",
      repeat_delay_);
  }

  // Open the PCAP dump file
  RCLCPP_INFO(private_nh->get_logger(), "Opening PCAP file \"%s\"", filename_.c_str());
  if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_) ) == nullptr) {
    RCLCPP_FATAL(private_nh->get_logger(), "Error opening Velodyne socket dump file.");
    return;
  }

  std::stringstream filter;
  if (!devip_str_.empty()) {            // using specific IP?
    filter << "src host " << devip_str_ << " && ";
  }

  filter << "udp dst port " << port;
  pcap_compile(
    pcap_, &pcap_packet_filter_,
    filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
}

/** destructor */
InputPCAP::~InputPCAP()
{
  pcap_close(pcap_);
}

/** @brief Get one velodyne packet. */
int InputPCAP::getPacket(velodyne_msgs::msg::VelodynePacket * pkt, const double time_offset)
{
  struct pcap_pkthdr * header;
  const u_char * pkt_data;

  while (true) {
    int res;

    if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {
      // Skip packets not for the correct port and from the
      // selected IP address.
      if (0 == pcap_offline_filter(
          &pcap_packet_filter_,
          header, pkt_data))
      {
        continue;
      }

      // Keep the reader from blowing through the file.
      if (!read_fast_) {
        packet_rate_.sleep();
      }

      ::memcpy(&pkt->data[0], pkt_data + 42, packet_size);
      (void)time_offset;
      // time_offset not considered here, as no synchronization required
      pkt->stamp = private_nh_->get_clock()->now();
      empty_ = false;
      return 0;                   // success
    }

    if (empty_) {               // no data in file?
      RCLCPP_WARN(
        private_nh_->get_logger(), "Error %d reading Velodyne packet: %s",
        res, pcap_geterr(pcap_));
      return -1;
    }

    if (read_once_) {
      RCLCPP_INFO(private_nh_->get_logger(), "end of file reached -- done reading.");
      return -1;
    }

    if (repeat_delay_ > 0.0) {
      RCLCPP_INFO(
        private_nh_->get_logger(), "end of file reached -- delaying %.3f seconds.",
        repeat_delay_);
      ::usleep(::rint(repeat_delay_ * 1000000.0));
    }

    RCLCPP_DEBUG(private_nh_->get_logger(), "replaying Velodyne dump file");
    // I can't figure out how to rewind the file, because it
    // starts with some kind of header.  So, close the file
    // and reopen it with pcap.
    pcap_close(pcap_);
    pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
    empty_ = true;              // maybe the file disappeared?
  }  // loop back and try again
}

}  // namespace velodyne_driver
