#ifndef VELODYNE_DRIVER_TIME_CONVERSION_HPP
#define VELODYNE_DRIVER_TIME_CONVERSION_HPP

#include <ros/ros.h>
#include <ros/time.h>

/** @brief Function used to check that hour assigned to timestamp in conversion is
 * correct. Velodyne only returns time since the top of the hour, so if the computer clock
 * and the velodyne clock (gps-synchronized) are a little off, there is a chance the wrong
 * hour may be associated with the timestamp
 * 
 * @param stamp timestamp recovered from velodyne
 * @param nominal_stamp time coming from computer's clock
 * @return timestamp from velodyne, possibly shifted by 1 hour if the function arguments
 * disagree by more than a half-hour.
 */
ros::Time resolveHourAmbiguity(const ros::Time &stamp, const ros::Time &nominal_stamp) {
    const int HALFHOUR_TO_SEC = 1800;
    ros::Time retval = stamp;
    if (nominal_stamp.sec > stamp.sec) {
        if (nominal_stamp.sec - stamp.sec > HALFHOUR_TO_SEC) {
            retval.sec = retval.sec + 2*HALFHOUR_TO_SEC;
        }
    } else if (stamp.sec - nominal_stamp.sec > HALFHOUR_TO_SEC) {
        retval.sec = retval.sec - 2*HALFHOUR_TO_SEC;
    }
    return retval;
}

ros::Time rosTimeFromGpsTimestamp(const uint8_t * const data) {
    const int HOUR_TO_SEC = 3600;
    // time for each packet is a 4 byte uint
    // It is the number of microseconds from the top of the hour
    uint32_t usecs = (uint32_t) ( ((uint32_t) data[3]) << 24 |
                                  ((uint32_t) data[2] ) << 16 |
                                  ((uint32_t) data[1] ) << 8 |
                                  ((uint32_t) data[0] ));
    ros::Time time_nom = ros::Time::now(); // use this to recover the hour
    uint32_t cur_hour = time_nom.sec / HOUR_TO_SEC;
    ros::Time stamp = ros::Time((cur_hour * HOUR_TO_SEC) + (usecs / 1000000),
                                (usecs % 1000000) * 1000);
    stamp = resolveHourAmbiguity(stamp, time_nom);
    return stamp;
}

#endif //VELODYNE_DRIVER_TIME_CONVERSION_HPP
