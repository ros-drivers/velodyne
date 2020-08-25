^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package velodyne_pcl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2020-07-09)
------------------
* Velodyne pcl (`#335 <https://github.com/ros-drivers/velodyne/issues/335>`_)
  * fix time assignment in organized cloud container
  * add velodyne_pcl package with point_types.h
  * rename containers to cover the added time property
  * Adding roslint to velodyne_pcl. (`#1 <https://github.com/ros-drivers/velodyne/issues/1>`_)
  * Update CMake version to 3.5
  * Update package.xml to Format2 and package version to 1.5.2
  Co-authored-by: Joshua Whitley <josh.whitley@autoware.org>
* Contributors: Sebastian Pütz

1.5.2 (2019-01-28)
------------------

1.5.1 (2018-12-10)
------------------

1.5.0 (2018-10-19)
------------------

1.4.0 (2018-09-19)
------------------

1.3.0 (2017-11-10)
------------------
* remove velodyne_pcl from velodyne stack trunk
* make INFO message on every pointcloud a DEBUG message
* finished reorganization + splitting up of the nodes + changed velodyne/pcl_points2 topic name to velodyne/pointcloud
* reorganized the package, and renamed ExamplePublisher to Publisher
* run transform2 nodelet
* fixed issues in nodelets, equated nodes to match nodelets - there was some issue with using KdTree in the nodelet that I did not have time to debug - so it has been removed
* created example nodelets for the corresponding nodes
* some pretty printing + fixed subscriber to do euclidean cluster extraction using PCL
* fixed bugs in example code
* added a couple of examples using the pcl_ros bridge
* add waitForTransform() before transforming point cloud packet
* change convertPoints() to use PointCloud2ConstPtr for input
* add nodelet to assign colors to laser rings for visualization
* remove redundant typedefs
* add transform2 nodelet unit test
* add PCL transform to transform2 nodelet
* check in omitted nodelets.xml update
* create initial transform2 nodelet from cloud2 nodelet
* nodelet test now works, remove "broken" label
* set svn:keywords props to Id
* publish PointCloud2 for each scan in nodelet
* Prepare cloud2 nodelet for accumulating data for a whole scan (incomplete).
  Add Velodyne::xyz_scans_t typedef.
* set Id property on test files
* add simple unit tests of node and nodelet publication
  rates -- nodelet currently fails the test
* remove cerr printing, nodelet now runs
* fixed nodelets.xml so that nodelet is correctly visible
* use new point_types header in nodelet, too
* Add exported header file for custom Velodyne point types.
  Use new velodyne::PointXYZIR custom point type in cloud2 node.
  Print std::cerr message when nodelet methods entered (nothing shows).
* fixed issue with custom pcl point type in cloud2_nodelet
* library path correction
* small nodelet improvements
* Make Velodyne:: namespace references explicit.
  Accumulate data using PointCloud<PointXYZI> instead of PointCloud2.
  Delete some unnecessary cruft.
* comment out enough so nodelet will at least compile
* add nodelets.xml export
* experimental stack for PCL output
* Contributors: austinrobot, jack.oquin, piyushk
