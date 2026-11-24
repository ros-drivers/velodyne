// Copyright 2019 Matthew Pitropov, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
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

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "velodyne_driver/time_conversion.hpp"

TEST(TimeConversion, BytesToTimestamp)
{
  auto clock_ = std::make_shared<rclcpp::Clock>();
  auto ros_stamp = clock_->now();

  double secs;
  double fractpart = std::modf(ros_stamp.seconds(), &secs);
  double nanosecs = fractpart * 1000000000;
  // get the seconds past the hour and multiply by 1million to convert to microseconds
  // divide nanoseconds by 1000 to convert to microseconds
  uint32_t since_the_hour = ((static_cast<int>(secs) % 3600) * 1000000) + (nanosecs / 1000);

  uint8_t native_format[4];
  native_format[0] = 0xFF & since_the_hour;
  native_format[1] = 0xFF & (((uint32_t)since_the_hour) >> 8);
  native_format[2] = 0xFF & (((uint32_t)since_the_hour) >> 16);
  native_format[3] = 0xFF & (((uint32_t)since_the_hour) >> 24);

  rclcpp::Time ros_stamp_converted = rosTimeFromGpsTimestamp(ros_stamp, native_format);

  ASSERT_NEAR(ros_stamp_converted.nanoseconds(), ros_stamp.nanoseconds(), 2000);
}

static void toNativeFormat(uint32_t usecs, uint8_t * native_format)
{
  native_format[0] = 0xFF & usecs;
  native_format[1] = 0xFF & (usecs >> 8);
  native_format[2] = 0xFF & (usecs >> 16);
  native_format[3] = 0xFF & (usecs >> 24);
}

// 2023-11-14 22:00:00 UTC, the top of an hour
static const int64_t kTopOfHourSec = 1700000000LL - (1700000000LL % 3600);

TEST(TimeConversion, PacketStampedJustBeforeHostHour)
{
  // The host clock has just passed the top of the hour, but the packet
  // was stamped 100 ms before it, at 3599.9 s past the previous hour.
  rclcpp::Time host_stamp(RCL_S_TO_NS(kTopOfHourSec) + RCL_MS_TO_NS(100));
  uint8_t native_format[4];
  toNativeFormat(3599900000U, native_format);

  rclcpp::Time converted = rosTimeFromGpsTimestamp(host_stamp, native_format);

  EXPECT_EQ(converted.nanoseconds(), RCL_S_TO_NS(kTopOfHourSec) - RCL_MS_TO_NS(100));
}

TEST(TimeConversion, PacketStampedJustAfterHostHour)
{
  // The host clock is 100 ms before the top of the hour, but the packet
  // was stamped 100 ms after it.
  rclcpp::Time host_stamp(RCL_S_TO_NS(kTopOfHourSec) - RCL_MS_TO_NS(100));
  uint8_t native_format[4];
  toNativeFormat(100000U, native_format);

  rclcpp::Time converted = rosTimeFromGpsTimestamp(host_stamp, native_format);

  EXPECT_EQ(converted.nanoseconds(), RCL_S_TO_NS(kTopOfHourSec) + RCL_MS_TO_NS(100));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
