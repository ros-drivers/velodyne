Velodyne ROS 2 pointcloud to laserscan converter
================================================

This is a ROS 2 package that takes pointcloud data as output by one of the velodyne_pointcloud nodes and converts it to a single laserscan.

Published Topics
----------------
* `scan` (`sensor_msgs/LaserScan`) - The laserscan that results from taking one line of the pointcloud.

Subscribed Topics
-----------------
* `velodyne_points` (`sensor_msgs/PointCloud2`) - The pointcloud that results from the raw velodyne data.

Parameters
----------
* `ring` (int) - The "ring" of the Velodyne to use for the single line.  If less than 0, a default ring per device will be used.  Defaults to -1.
* `resolution` (double) - The resolution in meters that each point provides.  Defaults to 0.007.
