Velodyne ROS 2 raw to pointcloud converters
===========================================

This is a ROS 2 package that takes raw velodyne data as output by the velodyne_driver node, and converts it into a `sensor_msgs/PointCloud2` message.

There are two different nodes here:

`velodyne_convert_node` - This node takes in the raw data and converts it to a pointcloud immediately.
`velodyne_transform_node` - This node takes the raw data and converts it only when a corresponding `tf` message with a compatible timestamp has arrived.

The topics and parameters for both nodes are identical, so they'll be described together below.

Published Topics
----------------
* `velodyne_points` (`sensor_msgs/PointCloud2`) - The pointcloud that results from the raw velodyne data.

Subscribed Topics
-----------------
* `velodyne_packets` (`velodyne_msgs/VelodyneScan`) - The raw velodyne packets coming from the velodyne_driver.

Parameters
----------
* `calibration` (string) - The path to the calibration file for the particular device.  There are a set of default calibration files to start with in the "params" subdirectory in this package.  Defaults to the empty string.
* `min_range` (double) - The minimum range in meters that a point must be to be added to the resulting point cloud.  Points closer than this are discarded.  Must be between 0.1 and 10.0.  Defaults to 0.9.
* `max_range` (double) - The maximum range in meters that a point must be to be added to the resulting point cloud.  Points further away than this are discarded.  Must be between 0.1 and 200.0.  Defaults to 130.0.
* `view_direction` (double) - The point around the circumference of the device, in radians, to "center" the view.  Combined with `view_width`, this allows the node to generate a pointcloud only for the given width, centered at this point.  This can vastly reduce the CPU requirements of the node.  Must be between -Pi and Pi, where 0 is straight ahead from the device.  Defaults to 0.0.
* `view_width` (double) - The width, in radians, of the view to generate for the resulting pointcloud.  Combined with `view_direction`, this allows the node to generate a pointcloud only for the given width, centered at the `view_direction` point.  This can vastly reduce the CPU requirements of the node.  Must be between 0 and 2*Pi.  Defaults to 2*Pi.
* `organize_cloud` (bool) - Whether to organize the cloud by ring (True), or to use the order as it comes directly from the driver (False).  Defaults to True.
* `target_frame` (string) - The coordinate frame to apply to the generated point cloud header before publishing.  If the empty string (the default), the frame is passed along from the driver packet.  If this frame is different than the `fixed_frame`, a transformation to this coordinate frame is performed while creating the pointcloud.
* `fixed_frame` (string) - The fixed coordinate frame to transform the data from.
