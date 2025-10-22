[![](https://github.com/ros-drivers/velodyne/workflows/Basic%20Build%20Workflow/badge.svg?branch=ros2)](https://github.com/ros-drivers/velodyne/actions)

Overview
========

Velodyne<sup>1</sup> is a collection of ROS<sup>2</sup> packages supporting `Velodyne high
definition 3D LIDARs`<sup>3</sup>.

**Warning**:

  The `<ros_distro>-devel` branch normally contains code being tested for the next
  ROS release.  It will not always work with every previous release.
  To check out the source for the most recent release, check out the
  tag `ros2-<version>` with the highest version number.

The current ``dashing-devel`` branch works with ROS Dashing.

- <sup>1</sup>Velodyne: http://www.ros.org/wiki/velodyne
- <sup>2</sup>ROS: http://www.ros.org
- <sup>3</sup>`Velodyne high definition 3D LIDARs`: http://www.velodynelidar.com/lidar/lidar.aspx


**ROS 2 Middleware (RMW) Note**:

For users encountering networking or performance issues with high-bandwidth sensors (such as data loss, discussed in [Issue #557](https://github.com/ros-drivers/velodyne/issues/557)), switching the default ROS 2 middleware may resolve the problem.

This package is compatible with alternative RMW implementations. You can follow the [official ROS 2 documentation](https://docs.ros.org/en/jazzy/Installation/RMW-Implementations.html) on how to install and select a different middleware.
