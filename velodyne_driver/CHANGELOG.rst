Change history
==============

2.4.0 (2023-05-27)
------------------
* Disable cut_angle by default. (`#497 <https://github.com/ros-drivers/velodyne/issues/497>`_)
* Unify tf frame parameters between transform and cloud nodes (`#344 <https://github.com/ros-drivers/velodyne/issues/344>`_) (`#453 <https://github.com/ros-drivers/velodyne/issues/453>`_)
  * Unify tf frame parameters between transform and cloud nodes
  At this point there is no need any more for cloud node because transform node includes all features of cloud node.
  Co-authored-by: AndreasR30 <andreas-reich@live.de>
  Co-authored-by: anre <andreas.reich@unibw.de>
* Contributors: Daisuke Nishimatsu, Joshua Whitley

2.3.0 (2022-07-08)
------------------
* Updating maintainer email address. (`#450 <https://github.com/ros-drivers/velodyne/issues/450>`_)
  * Updating maintainer email address.
  * chore: update maintainer email address
  Co-authored-by: Joshua Whitley <jwhitley@autonomoustuff.com>
* fix: use rclcpp logger instead of perror
  Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* reuse Velodyne UDP port (`#427 <https://github.com/ros-drivers/velodyne/issues/427>`_)
* Added config option to timestamp a full scan based on first velo packet instead of last packet (`#436 <https://github.com/ros-drivers/velodyne/issues/436>`_)
  Co-authored-by: Shawn Hanna <shawn@kaarta.com>
* Replace deprecated argument names in launch (`#430 <https://github.com/ros-drivers/velodyne/issues/430>`_)
* 2.1.1
* Updating for first Galactic release
* Minor fixes to string formatting. (`#396 <https://github.com/ros-drivers/velodyne/issues/396>`_)
  These changes will allow velodyne to compile without warnings
  on Rolling (soon to be Galactic).  The changes are also backwards
  compatible to Foxy if we want to backport them.
* Contributors: Chris Lalancette, Daisuke Nishimatsu, Joshua Whitley, Keane Quigley, Nagy Dániel Zoltán

2.1.1 (2021-08-23)
------------------
* Minor fixes to string formatting. (`#396 <https://github.com/ros-drivers/velodyne/issues/396>`_)
  These changes will allow velodyne to compile without warnings
  on Rolling (soon to be Galactic).  The changes are also backwards
  compatible to Foxy if we want to backport them.
* Contributors: Chris Lalancette

2.1.0 (2020-07-10)
------------------
* Fixing Foxy-specific uncrustify errors.
* Contributors: Joshua Whitley

2.0.0 (2020-07-10)
------------------
* Fix dependencies in package.xml (`#331 <https://github.com/ros-drivers/velodyne/issues/331>`_)
  Ensure that we depend on ament_cmake_ros as appropriate.
* Fixes pointed out by clang-tidy (`#310 <https://github.com/ros-drivers/velodyne/issues/310>`_)
  * Rearrange header includes so clang-tidy is happy.
  * Get rid of unnecessary void arguments on methods.
  * Properly mark methods as override where appropriate.
  * Initialize members and stack variables to zero before use.
  * Mark function implementations in header files 'inline'.
  * Use more efficient 'empty' method instead of empty string.
  * Get rid of unnecessary else statements after a continue/return.
  * Get rid of unnecessary == false use.
  * Make sure to add a virtual destructor to DataContainerBase.
  * Use string != comparison instead of 'compare' method.
  * Rename PointcloudXYZIR parameter name to match implementation.
  * Get rid of usage of typedef.
  * Be more explicit about using floats.
  * Do an explicit lround to do int->float conversion.
  * Fix the intensity calculation.
  * Remove the azimuth from addPoint.
  * Make sure to reset the cloud data to 0 before reusing.
  * Use underscores on pointcloud member variables.
  * Using std::lround means we don't need to add 0.5 to the result.
  * Slightly rearrange and simplify range checks.
  * Add in example launch files for VLP32C.
* Dashing fixes (`#307 <https://github.com/ros-drivers/velodyne/issues/307>`_)
  * Use std::make_unique as appropriate.
  * Fix the azimuth calculation.
  * Fix a crash during driver startup.
  * Always recalculate the row_step while setting up the cloud.
  * Make sure to call computeTransformation.
  * Fix style errors pointed out by flake8.
* ROS2: Add Linters to velodyne_driver (`#301 <https://github.com/ros-drivers/velodyne/issues/301>`_)
  * Renaming files to match ROS2 conventions.
  * Cleaning up velodyne_driver and fixing diags.
  * velodyne_driver linting complete.
  * VD: Removing unused namespace.
* Add in example launch files.
* Disable copy, move, and assign operators.
* Build the components so the velodyne can be run as a component.
* Add in NodeOptions to node constructors.
* Throw an exception for an invalid Velodyne model.
* Add explicit, final, and override to classes where appropriate.
* Switch to using nullptr everywhere.
* Pass read_once, read_fast, and repeat_delay to InputPCAP constructor.
* Change filename parameter to const reference.
* Pass the devip string into the Input* constructors.
* Move gps_time into the InputSocket class.
* Get rid of default parameters for Input* constructors.
* Rename nodes.
* Update the documentation for ROS 2.
* Switch to using the cut angle by default.
* Merge pull request `#251 <https://github.com/ros-drivers/velodyne/issues/251>`_ from clalancette/dashing-devel3
  ROS 2 Dashing port
* Merge pull request `#214 <https://github.com/ros-drivers/velodyne/issues/214>`_ from spuetz/feature/opc_nopcl
  Container cleanup and organized pointclouds
* Merge pull request `#243 <https://github.com/ros-drivers/velodyne/issues/243>`_ from dandedrick/reconfig-enable
  driver: add enabled reconfigure param
* Merge pull request `#234 <https://github.com/ros-drivers/velodyne/issues/234>`_ from kmhallen/c++11
  Set minimum C++ standard to C++11
* Merge pull request `#231 <https://github.com/ros-drivers/velodyne/issues/231>`_ from ros-drivers/ci/test_better_output
  CI: Adding roslint as separate step. Limiting output of catkin build.
* Merge pull request `#227 <https://github.com/ros-drivers/velodyne/issues/227>`_ from ros-drivers/roslint
  Applying roslint to velodyne_pointcloud.
* Merge pull request `#225 <https://github.com/ros-drivers/velodyne/issues/225>`_ from ros-drivers/roslint
  Adding roslint to velodyne_driver.
* Merge pull request `#222 <https://github.com/ros-drivers/velodyne/issues/222>`_ from mpitropov/feat_Use_GPS_time
  Add flag to enable using GPS time from within the Velodyne packet instead of ROS time for scan.
* Merge pull request `#220 <https://github.com/ros-drivers/velodyne/issues/220>`_ from nbussas/static
  Transform static variable into member
* Merge pull request `#216 <https://github.com/ros-drivers/velodyne/issues/216>`_ from ros-drivers/maint/poll_timeout_handling
  Testing reporting error instead of stopping node on disconnect.
* Contributors: Andreas Klintberg, Chris Lalancette, Dan Dedrick, Joshua Whitley, Kevin Hallenbeck, Matthew Pitropov, Nils Hauke Bussas, Sebastian, Sebastian Pütz

1.5.2 (2019-01-28)
------------------
* Merge pull request `#212 <https://github.com/ros-drivers/velodyne/issues/212>`_ from ros-drivers/maint/vdump_as_root
  Modifying vdump script for use as root.
  Tested by @andersfischernielsen.
* Merge pull request `#205 <https://github.com/ros-drivers/velodyne/issues/205>`_ from xiesc/master
  support for 64E-S3
* Contributors: Joshua Whitley, Shichao XIE, xiesc

1.5.1 (2018-12-10)
------------------

1.5.0 (2018-10-19)
------------------
* Merge pull request `#187 <https://github.com/ros-drivers/velodyne/issues/187>`_ from moooeeeep/master
  Fixed sign error in return value of InputSocket::getPacket()
* bugfix: getPacket() function is expected to return negative value on error
* Contributors: Fabian Maas, Joshua Whitley

1.4.0 (2018-09-19)
------------------
* Merge pull request `#178 <https://github.com/ros-drivers/velodyne/issues/178>`_ from sts-thm/bugfix_issue\_`#174 <https://github.com/ros-drivers/velodyne/issues/174>`_
  Bugfix issue `#174 <https://github.com/ros-drivers/velodyne/issues/174>`_
* Removed debug outputs
* Changes fixing deadlock for specific cut_angle values.
* Merge pull request `#135 <https://github.com/ros-drivers/velodyne/issues/135>`_ from cfneuhaus/bugfix
  Bugfix: when no device ip is set, we still want to filter by udp port.
* Merge pull request `#170 <https://github.com/ros-drivers/velodyne/issues/170>`_ from ros-drivers/maint/move_header_files
  Moving header files to traditional location inside include folders.
* Merge pull request `#160 <https://github.com/ros-drivers/velodyne/issues/160>`_ from ros-drivers/maint/updating_package_xml_to_v2
* Updated all package.xmls to ver 2. Cleaned up catkin_lint errors.
  All package.xml files are now compatible with version 2 of the
  package.xml specification in REP 140. Removed some unnecessary
  execute permissions on a few files. Fixed a missing test_depend.
* Merge pull request `#151 <https://github.com/ros-drivers/velodyne/issues/151>`_ from Axel13fr/feature/No_Communication_Diag_Update
* Fix packet rate for the Velodyne 32C
* Conventions: adding name for unused method parameter.
* Added a periodic update of the diagnostics so that when no data is received at all from the Velodyne, a diagnostic information will be published. The previous implementation would publish diagnostics only on packet reception.
* Merge pull request `#139 <https://github.com/ros-drivers/velodyne/issues/139>`_ from ASDeveloper00/vlp32
  Adding support for VLP-32C.
* Merge pull request `#138 <https://github.com/ros-drivers/velodyne/issues/138>`_ from volkandre/cut_at_specified_angle_feature
* cut_angle parameter is now in rad according to REP 103
* Fixed timestamp related bug found by @cfneuhaus, which was described here: https://github.com/ros-drivers/velodyne/pull/126#discussion_r154137793
* bugfix: when no device ip is set, we still want to filter by udp port.
* Contributors: Andre Volk, CNR, Denis Dillenberger, Frank Neuhaus, Jack O'Quin, Joshua Whitley, Sammy Pfeiffer, Tobias Athmer, axd, kennouni

1.3.0 (2017-11-10)
------------------
* Merge pull request `#129 <https://github.com/ros-drivers/velodyne/issues/129>`_ from kmhallen/pluginlib_macro
  Modern pluginlib macro
* Update to use non deprecated pluginlib macro
* add launch args to support multiple devices (`#108 <https://github.com/ros-drivers/velodyne/issues/108>`_)
* Merge pull request `#101 <https://github.com/ros-drivers/velodyne/issues/101>`_ from teosnare/master
  velodyne_driver/src/lib/input.cc : fix for device_ip filter
* Merge pull request `#104 <https://github.com/ros-drivers/velodyne/issues/104>`_ from altrouge/launch_options
  Add more options in launch files.
* Rearranged alphabetically.
* Add more options in launch files.
  - rpm, device_ip, port, read_once, read_fast, repeat_delay
* velodyne_driver/src/lib/input.cc : fix for device_ip filter
  Fix for device_ip filter in InputSocket: initialization of devip\_ for correct ip filtering in InputSocket::getPacket.
* velodyne_driver: credit @priyankadey for VLP-16 bug fix (`#96 <https://github.com/ros-drivers/velodyne/issues/96>`_)
* Merge pull request `#96 <https://github.com/ros-drivers/velodyne/issues/96>`_ from priyankadey/master
  updated VLP-16 packet rate from user manual.
* updated VLP-16 packet rate from user manual.
  Also verified with sensor. It reduced overlap in the pointcloud
* update change history
* Merge pull request `#94 <https://github.com/ros-drivers/velodyne/issues/94>`_ from ros-drivers/pcap_port
  velodyne_driver: use port number for PCAP data (`#46 <https://github.com/ros-drivers/velodyne/issues/46>`_, `#66 <https://github.com/ros-drivers/velodyne/issues/66>`_)
* fix g++ 5.3.1 compile errors (`#94 <https://github.com/ros-drivers/velodyne/issues/94>`_)
* merge current master (`#94 <https://github.com/ros-drivers/velodyne/issues/94>`_)
* Merge pull request `#91 <https://github.com/ros-drivers/velodyne/issues/91>`_ from chukcha2/master
  update velodyne_driver package description to include all models
* update velodyne_driver package description to include all models
* Merge pull request `#89 <https://github.com/ros-drivers/velodyne/issues/89>`_ from Tones29/feat_dynrec_driver
  Add dynamic latency configuration to velodyne_driver
* velodyne_driver: Add dynamic_reconfigure and time_offset correction
  The value of time_offset is added to the calculated time stamp in live mode for each packet.
* velodyne_driver: Make input destructors virtual
* prepare change history for coming Indigo release (`#59 <https://github.com/ros-drivers/velodyne/issues/59>`_)
* velodyne_driver: use port number for PCAP data (`#66 <https://github.com/ros-drivers/velodyne/issues/66>`_)
* Merge pull request `#39 <https://github.com/ros-drivers/velodyne/issues/39>`_ from zooxco/multivelodyne
  support for multiple velodynes
* Merge pull request `#44 <https://github.com/ros-drivers/velodyne/issues/44>`_ from SISegwayRmp/master
  adding driver and pointcloud support for the VLP16
* adding the VLP16 test scripts and updating the CMakeLists to include the test file from http://download.ros.org/data/velodyne/vlp16.pcap
* adding support for the VLP16
* parameters to set the udp port
* fixed missing header
* cleanup debug line
* parameter and code added for working with multiple velodynes
* Contributors: Andreas Wachaja, Brice Rebsamen, Daniel Jartoux, Denis Dillenberger, Gabor Meszaros, Ilya, Jack O'Quin, Joshua Whitley, Kevin Hallenbeck, Matteo Murtas, Micho Radovnikovich, Priyanka Dey, William Woodall, jack.oquin, junior, phussey

1.2.0 (2014-08-06)
------------------
* Fixed bug in diagnostic rate for driver (`#16
  <https://github.com/ros-drivers/velodyne/issues/16>`_)
* Contributors: Brice Rebsamen, Jack O'Quin

1.1.2 (2013-11-05)
-------------------

 * Move unit test data to download.ros.org (`#18`_).
 * Install missing vdump script (`#17`_).

1.1.1 (2013-07-30)
------------------

 * Add support for HDL-64E S2 and S2.1 models, which were not working before (`#11`_), thanks to Gabor Meszaros (`#12`_).
 * Add additional parameters to launch files (`#14`_).

1.1.0 (2013-07-16)
------------------

 * Fix build problems due to PCL 1.7 API incompatibilities (`#8`_),
   thanks to William Woodall.  This version also works with Groovy, as
   long as the correct ``pcl_conversions`` is installed.
 * Fix errors with Mac OSX compiler (`#8`_).
 * Install ``pluginlib`` XML files (`#9`_).
 * Install some launch and parameter files.
 * Enable unit tests when ``CATKIN_ENABLE_TESTING`` is set (`#10`_).

1.0.1 (2013-06-15)
------------------

 * Declare explicit ``pluginlib`` dependency (`#4`_).

1.0.0 (2013-06-14)
------------------

 * Convert to catkin (`#1`_).
 * Release to Hydro.

0.9.2 (2013-07-08)
------------------

 * Fix Groovy build problem (`#7`_).

0.9.1 (2012-06-05)
------------------

 * Driver socket read path improvements.
 * Add unit tests with 32E data.
 * Released to Electric, Fuerte and Groovy.

0.9.0 (2012-04-03)
------------------

 * Completely revised API, anticipating a 1.0.0 release.
 * HDL-32E device support.
 * New velodyne_driver and velodyne_pointcloud packages.
 * Old velodyne_common and velodyne_pcl packages no longer included.
 * Released to Electric, Fuerte and Groovy.

0.2.6 (2011-02-23)
------------------

 * Label all timing-dependent tests "realtime" so they do not run by
   default on the build farm machines.

0.2.5 (2010-11-19)
------------------

 * Initial implementation of new 0.3 interfaces.
 * Support for ROS 1.3 `std_msgs::Header` changes.

0.2.0 (2010-08-17)
------------------

 * Initial release to ROS C-turtle.

.. _`#1`: https://github.com/ros-drivers/velodyne/issues/1
.. _`#4`: https://github.com/ros-drivers/velodyne/issues/4
.. _`#7`: https://github.com/ros-drivers/velodyne/issues/7
.. _`#8`: https://github.com/ros-drivers/velodyne/pull/8
.. _`#9`: https://github.com/ros-drivers/velodyne/issues/9
.. _`#10`: https://github.com/ros-drivers/velodyne/issues/10
.. _`#11`: https://github.com/ros-drivers/velodyne/issues/11
.. _`#12`: https://github.com/ros-drivers/velodyne/pull/12
.. _`#13`: https://github.com/ros-drivers/velodyne/issues/13
.. _`#14`: https://github.com/ros-drivers/velodyne/pull/14
.. _`#17`: https://github.com/ros-drivers/velodyne/issues/17
.. _`#18`: https://github.com/ros-drivers/velodyne/issues/18
.. _`#20`: https://github.com/ros-drivers/velodyne/issues/20
