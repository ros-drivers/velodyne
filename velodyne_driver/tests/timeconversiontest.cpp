#include "velodyne_driver/time_conversion.hpp"
#include <ros/time.h>
#include <gtest/gtest.h>

TEST(TimeConversion, BytesToTimestamp) {
    ros::Time::init();
    ros::Time ros_stamp = ros::Time::now();

    // get the seconds past the hour and multiply by 1million to convert to microseconds
    // divide nanoseconds by 1000 to convert to microseconds
    uint32_t since_the_hour = ((ros_stamp.sec % 3600) * 1000000) + ros_stamp.nsec / 1000;

    uint8_t native_format[4];
    native_format[0] = 0xFF & since_the_hour;
    native_format[1] = 0xFF & (((uint32_t)since_the_hour) >> 8);
    native_format[2] = 0xFF & (((uint32_t)since_the_hour) >> 16);
    native_format[3] = 0xFF & (((uint32_t)since_the_hour) >> 24);

    ros::Time ros_stamp_converted = rosTimeFromGpsTimestamp(native_format);

    ASSERT_EQ(ros_stamp_converted.sec, ros_stamp.sec);
    ASSERT_NEAR(ros_stamp_converted.nsec, ros_stamp.nsec, 1000);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}
