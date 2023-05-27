Change history
==============

2.4.0 (2023-05-27)
------------------
* Add invalid points in organized cloud (`#360 <https://github.com/ros-drivers/velodyne/issues/360>`_) (`#492 <https://github.com/ros-drivers/velodyne/issues/492>`_)
  * Set NaN in ordered point cloud in case of no return
  * Adapt to current master
  * consider min/max angle and timing_offsets also in organized mode
  Co-authored-by: Sebastian Scherer <sebastian.scherer2@de.bosch.com>
  Co-authored-by: Nuernberg Thomas (CR/AEV4) <thomas.nuernberg@de.bosch.com>
* Fixed row_step=0 when init_width=0 (dense cloud) (`#404 <https://github.com/ros-drivers/velodyne/issues/404>`_) (`#494 <https://github.com/ros-drivers/velodyne/issues/494>`_)
  The PointCloud2 msg will be then valid: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
  Referred issues:
  * https://answers.ros.org/question/377470/rtabmap-icp_odometrycpp453callbackcloud-fatal-error/
  * http://official-rtab-map-forum.67519.x6.nabble.com/Condition-scan3dMsg-data-size-scan3dMsg-row-step-scan3dMsg-height-not-met-td7852.html
  Co-authored-by: matlabbe <matlabbe@gmail.com>
* Unify tf frame parameters between transform and cloud nodes (`#344 <https://github.com/ros-drivers/velodyne/issues/344>`_) (`#453 <https://github.com/ros-drivers/velodyne/issues/453>`_)
  * Unify tf frame parameters between transform and cloud nodes
  At this point there is no need any more for cloud node because transform node includes all features of cloud node.
  Co-authored-by: AndreasR30 <andreas-reich@live.de>
  Co-authored-by: anre <andreas.reich@unibw.de>
* fix: modify some tests (`#452 <https://github.com/ros-drivers/velodyne/issues/452>`_)
  * Fixing new linter errors.
  * Changing command for running tests.
  * remove relative file path
  * remove unnecessary main
  * fix: restore hdl64e s2 float intensities test
  Co-authored-by: Joshua Whitley <jwhitley@autonomoustuff.com>
* Contributors: Daisuke Nishimatsu

2.3.0 (2022-07-08)
------------------
* Passing fixed_frame and target_frame to Convert object. (`#330 <https://github.com/ros-drivers/velodyne/issues/330>`_) (`#451 <https://github.com/ros-drivers/velodyne/issues/451>`_)
  Co-authored-by: Joshua Whitley <jwhitley@autonomoustuff.com>
* Updating maintainer email address. (`#450 <https://github.com/ros-drivers/velodyne/issues/450>`_)
  * Updating maintainer email address.
  * chore: update maintainer email address
  Co-authored-by: Joshua Whitley <jwhitley@autonomoustuff.com>
* Add per point time field (`#440 <https://github.com/ros-drivers/velodyne/issues/440>`_)
  * Initial commit to timestamp each point using the timing spec in the manuals
  * Added model param to each of the cloud nodelet starters
  * Minor cleanup. added author tag to rawdata
  * Move timing offsets functionality into class private. Also fix linter errors
  * added #include <vector> for linter
  * fix: suppress compiler warning
  * feat: change XYZIR to XYZIRT
  Co-authored-by: Shawn Hanna <shawn@kaarta.com>
* Link against yaml-cpp (`#443 <https://github.com/ros-drivers/velodyne/issues/443>`_)
* Increase the max_range of the 32C launch file (`#323 <https://github.com/ros-drivers/velodyne/issues/323>`_) (`#441 <https://github.com/ros-drivers/velodyne/issues/441>`_)
  Co-authored-by: Shawn Hanna <50845122+kaarta-SHanna@users.noreply.github.com>
* fix for `#267 <https://github.com/ros-drivers/velodyne/issues/267>`_, transform each packet (`#438 <https://github.com/ros-drivers/velodyne/issues/438>`_)
  * fix for `#267 <https://github.com/ros-drivers/velodyne/issues/267>`_, transform each packet
  * fix: fix scope of access modifiers
  Co-authored-by: Sebastian <spuetz@uos.de>
* Replace deprecated argument names in launch (`#430 <https://github.com/ros-drivers/velodyne/issues/430>`_)
* 2.1.1
* Updating for first Galactic release
* Contributors: Daisuke Nishimatsu, Joshua Whitley, Keane Quigley, Stephan Sundermann

2.1.1 (2021-08-23)
------------------

2.1.0 (2020-07-10)
------------------

2.0.0 (2020-07-10)
------------------
* More fixing of dependencies. (`#333 <https://github.com/ros-drivers/velodyne/issues/333>`_)
  * More fixing of dependencies.
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
* A few more minor style updates. (`#308 <https://github.com/ros-drivers/velodyne/issues/308>`_)
  - Get rid of the last users of new in the codebase
  - Get rid of the users of 'using' in the codebase
  - Rename 'calibrationFile' -> 'calibration_file'
* Dashing fixes (`#307 <https://github.com/ros-drivers/velodyne/issues/307>`_)
  * Use std::make_unique as appropriate.
  * Fix the azimuth calculation.
  * Fix a crash during driver startup.
  * Always recalculate the row_step while setting up the cloud.
  * Make sure to call computeTransformation.
  * Fix style errors pointed out by flake8.
* ROS2: Add Linters to velodyne_pointcloud (`#304 <https://github.com/ros-drivers/velodyne/issues/304>`_)
  * VP: Renaming files.
  * VP: Fixed copyrights.
  * VP: cpplint is satisfied!
  * VP: Converted gen_calibration.py to Python3.
  * Uncrustify fixed!
  * VP: Lint errors clear.
  * VP: Moving organized_cloud and pointcloud to lib and building as library.
* Check for intra-process subscribers
* Add in example launch files.
* Disable copy, move, and assign operators.
* Build the components so the velodyne can be run as a component.
* Add in NodeOptions to node constructors.
* Only find the "common" component to PCL.
* Make sure to install the header files for laserscan and pointcloud.
* Properly setup the row_step.
* Add explicit, final, and override to classes where appropriate.
* Remove organize_cloud, fixed_frame, and target_frame configs.
* Stop storing view\_{direction,width} in the Node classes.
* Stop storing {min,max}_range in the Node classes.
* Make DataContainerBase::configure const reference string parameters.
* Switch argument order to always be (min,max)
* Remove tmp\_{min,max}_angle from stored RawData config.
* Remove unused tf_ptr.
* Switch container_ptr to a unique_ptr.
* container_ptr -> container_ptr\_
* Don't store number of lasers in the Transfrom/Convert node.
* Make the RawData class RAII-style.
* Make RawData::setup not return the number of lasers.
* Remove unused laser_corrections_maps.
* Remove the calibration read method.
* Make the Calibration class only have the RAII-style constructor.
* Remove unnecessary ros_info parameter from Calibration.
* Remove unused calibration write method.
* Remove unused setupOffline method.
* Remove calibrationFile from rawdata config.
* Switch to using nullptr everywhere.
* fixed timpepoint issue
* Remove dead store from rawdata.cc
* Rename nodes.
* Switch organize_cloud to true by default.
* Update the documentation for ROS 2.
* Remove debugging statements.
* Fix dead pointers in calibration.cc.
* Merge pull request `#251 <https://github.com/ros-drivers/velodyne/issues/251>`_ from clalancette/dashing-devel3
  ROS 2 Dashing port
* Merge pull request `#214 <https://github.com/ros-drivers/velodyne/issues/214>`_ from spuetz/feature/opc_nopcl
  Container cleanup and organized pointclouds
* Merge pull request `#236 <https://github.com/ros-drivers/velodyne/issues/236>`_ from mpitropov/fix_transform_node_frame_bug
  set correct output frame
* Merge pull request `#234 <https://github.com/ros-drivers/velodyne/issues/234>`_ from kmhallen/c++11
  Set minimum C++ standard to C++11
* Merge pull request `#231 <https://github.com/ros-drivers/velodyne/issues/231>`_ from ros-drivers/ci/test_better_output
  CI: Adding roslint as separate step. Limiting output of catkin build.
* Merge pull request `#227 <https://github.com/ros-drivers/velodyne/issues/227>`_ from ros-drivers/roslint
  Applying roslint to velodyne_pointcloud.
* Merge pull request `#223 <https://github.com/ros-drivers/velodyne/issues/223>`_ from mpitropov/feat_Add_fixed_frame
  Add fixed frame and use ros message time within transform node
* Merge pull request `#224 <https://github.com/ros-drivers/velodyne/issues/224>`_ from mpitropov/feat_add_diagnostics
  Added diagnostic publishing
* Merge pull request `#222 <https://github.com/ros-drivers/velodyne/issues/222>`_ from mpitropov/feat_Use_GPS_time
  Add flag to enable using GPS time from within the Velodyne packet instead of ROS time for scan.
* Contributors: Andreas Klintberg, Chris Lalancette, Ian Colwell, Joshua Whitley, Kevin Hallenbeck, Matthew Pitropov, P. J. Reed, Sebastian, Sebastian Pütz

1.5.2 (2019-01-28)
------------------
* Merge pull request `#205 <https://github.com/ros-drivers/velodyne/issues/205>`_ from xiesc/master
  support for 64E-S3
* add an example yaml file for S3
* Contributors: Joshua Whitley, Shichao XIE, xiesc

1.5.1 (2018-12-10)
------------------
* Merge pull request `#194 <https://github.com/ros-drivers/velodyne/issues/194>`_ from ros-drivers/avoid_unnecessary_computation
  Avoid unnecessary computation - causes approximately 20% performance increase on VLP-32C - should be similar for other sensors
* std::vector<>::reserve is your friend
* add static to avoid frequence memory allocation
* avoid unecesary calculations in unpack()
* Contributors: Davide Faconti, Joshua Whitley

1.5.0 (2018-10-19)
------------------
* Merge pull request `#164 <https://github.com/ros-drivers/velodyne/issues/164>`_ from ros-drivers/maint/vlp_32c_support
  Adding VLP-32C support.
  This was tested by AutonomouStuff and several external users. Though it does not include new information that I've learned (it appears that the distance resolution is different <50m vs >=50m), it is a good start.
* Merge pull request `#189 <https://github.com/ros-drivers/velodyne/issues/189>`_ from kveretennicov/patch-1
* Fix malformed plugin description XML
  ROS pluginlib only recognizes multiple <library> elements if they are under
  <class_libraries> XML root. It silently ignores malformed XMLs with multiple
  <library> "root"s and just reads the first one, due to relaxed way tinyxml2 does
  parsing. Though if you do `rosrun nodelet declared_nodelets`, the issue is
  reported properly.
  See also similar issue in https://github.com/ros-perception/perception_pcl/issues/131
* Adding distance_resolution to test yaml files.
* Adding VLP-32C support.
  Based on work done by @rockcdr. Adds distance_resolution calibration
  value to support 0.004m distance resolution for VLP-32C.
* Contributors: Joshua Whitley, Konstantin Veretennicov

1.4.0 (2018-09-19)
------------------
* Merge pull request `#178 <https://github.com/ros-drivers/velodyne/issues/178>`_ from sts-thm/bugfix_issue\_`#174 <https://github.com/ros-drivers/velodyne/issues/174>`_
  Bugfix issue `#174 <https://github.com/ros-drivers/velodyne/issues/174>`_
* Merge pull request `#177 <https://github.com/ros-drivers/velodyne/issues/177>`_ from C-NR/feature/WrapPointcloudData
  Feature/wrap pointcloud data
* Changes fixing deadlock for specific cut_angle values.
* moved definition of VPoint and VPointCloud back to namespace rawdata in rawdata.h
* put a wrapper around pointcloud data including a generic setter method to enable the use of arbitrary data structures  (pcl pointcloud, depth image, octomaps and so on) to be filled by just using RawData::unpack method with the wrapper object as parameter
* Merge pull request `#170 <https://github.com/ros-drivers/velodyne/issues/170>`_ from ros-drivers/maint/move_header_files
  Moving header files to traditional location inside include folders.
* Merge pull request `#160 <https://github.com/ros-drivers/velodyne/issues/160>`_ from ros-drivers/maint/updating_package_xml_to_v2
* Updated all package.xmls to ver 2. Cleaned up catkin_lint errors.
  All package.xml files are now compatible with version 2 of the
  package.xml specification in REP 140. Removed some unnecessary
  execute permissions on a few files. Fixed a missing test_depend.
* Merge pull request `#136 <https://github.com/ros-drivers/velodyne/issues/136>`_ from stsundermann/patch-1
  Use std::abs instead of abs
* Adding missing 32C configuration file.
* Merge pull request `#139 <https://github.com/ros-drivers/velodyne/issues/139>`_ from ASDeveloper00/vlp32
  Adding support for VLP-32C.
* Merge pull request `#138 <https://github.com/ros-drivers/velodyne/issues/138>`_ from volkandre/cut_at_specified_angle_feature
  Cut at specified angle feature
* Updated default cut_angle parameters in launch files after switching from deg to rad.
* Use std::abs instead of abs
  abs is the c version which returns an integer. This is probably not intended here, so use the templated std::abs function.
* Contributors: Andre Volk, Autonomoustuff Developer, CNR, Joshua Whitley, Kyle Rector, Stephan Sundermann, Tobias Athmer, kennouni

1.3.0 (2017-11-10)
-------------------
* Merge pull request `#110 <https://github.com/ros-drivers/velodyne/issues/110>`_ from kmhallen/master
  Added velodyne_laserscan package
* Merge remote-tracking branch ros-drivers/master
* Merge pull request `#129 <https://github.com/ros-drivers/velodyne/issues/129>`_ from kmhallen/pluginlib_macro
  Modern pluginlib macro
* Update to use non deprecated pluginlib macro
* Merge pull request `#127 <https://github.com/ros-drivers/velodyne/issues/127>`_ from swri-robotics/add_vlp16_hires_support
  Add VLP16 Puck Hi-Res config file
* Add VLP16 Puck Hi-Res support
* velodyne_pointcloud: remove incorrect catkin_package() DEPENDS option (`#93 <https://github.com/ros-drivers/velodyne/issues/93>`_)
  This eliminates a CMake warning when building on Xenial.
* Merge pull request `#111 <https://github.com/ros-drivers/velodyne/issues/111>`_ from OrebroUniversity/master
  Added an interface to set up raw data processing offline
* Added an interface to set up raw data processing from a locally defined calibration file. This method is useful when processing data offline from a bag file, without starting any ros master
* Added velodyne_laserscan package and inserted into existing launch files
* test multiple nodelet manager support (`#108 <https://github.com/ros-drivers/velodyne/issues/108>`_)
* add launch args to support multiple devices (`#108 <https://github.com/ros-drivers/velodyne/issues/108>`_)
* Merge pull request `#105 <https://github.com/ros-drivers/velodyne/issues/105>`_ from fudger/patch-1
  Remove unused constants.
* Merge pull request `#104 <https://github.com/ros-drivers/velodyne/issues/104>`_ from altrouge/launch_options
  Add more options in launch files.
* Rearranged alphabetically.
* Remove unused constants.
  DISTANCE_MAX and DISTANCE_MAX_UNITS are not used anywhere in the code.
  Furthermore, using them would lead to errors as both VLP-64 manuals state that returns above 120 m should not be used. The VLP-32 manual allows 70 m as the maximum valid sensor range.
* Merge pull request `#103 <https://github.com/ros-drivers/velodyne/issues/103>`_ from fudger/patch-1
  Fix misleading typecasts.
* Add more options in launch files.
  - rpm, device_ip, port, read_once, read_fast, repeat_delay
* Fix misleading typecasts.
  intensity and VPoint::intensity are both of type float.
* update change history
* merge current master (`#94 <https://github.com/ros-drivers/velodyne/issues/94>`_)
* Merge pull request `#92 <https://github.com/ros-drivers/velodyne/issues/92>`_ from adasta/master
  GCC Build Warnings
* Modified velodyne_point_cloud/src/lib/rawdata.cc to address warning
  that last_azimuth_diff variable may be used uninitialized.  Variable
  is now initialized to 0 at creation.
  velodyne/velodyne_pointcloud/src/lib/rawdata.cc:328:57: error: ‘last_azimuth_diff’ may be used uninitialized in this function [-Werror=maybe-uninitialized]
  azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
* Modified  velodyne_pointcloud/src/conversion/colors.cc to remove
  address build warning for strict-aliasing.
  velodyne/velodyne_pointcloud/src/conversions/colors.cc:84:58:
* Merge pull request `#89 <https://github.com/ros-drivers/velodyne/issues/89>`_ from Tones29/feat_dynrec_driver
  Add dynamic latency configuration to velodyne_driver
* velodyne_pointcloud: Fix compile warning "Wrong initialization order"
* velodyne_pointcloud: add dynamic reconfig update to change log (`#78 <https://github.com/ros-drivers/velodyne/issues/78>`_)
* Merge branch fudger-reconfigure_transform_node
* velodyne_pointcloud: use recommended add_dependencies() CMake variable `#78 <https://github.com/ros-drivers/velodyne/issues/78>`_
* velodyne_pointcloud: fix transform unit tests
  Use tf2_ros static_transform_publisher for more consistent timing (`#2 <https://github.com/ros-drivers/velodyne/issues/2>`_)
* Merge branch reconfigure_transform_node of https://github.com/fudger/velodyne
* prepare change history for coming Indigo release (`#59 <https://github.com/ros-drivers/velodyne/issues/59>`_)
* calibration: unit test case improvements (`#84 <https://github.com/ros-drivers/velodyne/issues/84>`_)
* calibration: read all intensities as float, then convert (`#84 <https://github.com/ros-drivers/velodyne/issues/84>`_)
* calibration: add gtest for `#84 <https://github.com/ros-drivers/velodyne/issues/84>`_
  This currently fails on 64e_s2.1-sztaki.yaml and on issue_84_float_intensities.yaml.
* calibration: make max_intensity and min_intensity optional (`#84 <https://github.com/ros-drivers/velodyne/issues/84>`_)
  This fixes a regression in the 32e and VLP-16 calibrations which do not contain
  intensity values. There is still a problem with the 64e_s2.1 calibration.
* Merge pull request `#76 <https://github.com/ros-drivers/velodyne/issues/76>`_ from pomerlef/master
  Sign inversion in some equations
* Merge pull request `#82 <https://github.com/ros-drivers/velodyne/issues/82>`_ from ros-drivers/fix_pr_80
  Fix pr 80; adding travis CI tests.
* fix the yaml-cpp 0.5 code paths
* Merge pull request `#80 <https://github.com/ros-drivers/velodyne/issues/80>`_ from ros-drivers/fix_yaml_import
  allow floats in min/max_intensity and make horiz_offset_correction optional
* allow horiz_offset_correction to be optional with 0 as default
* allow floats instead of ints in min/max_intensity
* Resolve frame ID name using tf prefix.
* Improve coding style.
* Set up dynamic reconfiguration for transform_node.
  Previously, transform_node has neither read parameters other than frame_id from the command line nor has it exposed these parameters via dynamic reconfigure. As parameters like max_range and view_width have been initialized to zero, the inconfigurable transform_node has returned an empty point cloud.
  Now, transform_node launches an reconfigure server just as cloud_node. In contrast to cloud_node, transform node exposes another parameter for dynamic reconfiguration: frame_id, i.e. the frame of reference the incoming Velodyne points are transformed to.
* Merge pull request `#77 <https://github.com/ros-drivers/velodyne/issues/77>`_ from fudger/pretty_print
  Fix output of calibration data onto console
* Add a missing space.
* Fix line that always indicates use of model VLP-16.
* Align console output of calibration data.
* Merge branch master of https://github.com/ros-drivers/velodyne
* resolve sign error
* Merge pull request `#73 <https://github.com/ros-drivers/velodyne/issues/73>`_ from fudger/master
  Correct important data type error for VLP-16
* Fix data type error that distorts the point cloud.
* Fix and add a few comments.
* Merge pull request `#68 <https://github.com/ros-drivers/velodyne/issues/68>`_ from jlblancoc/patch-1
  Remove unused variable
* Remove unused variable
  I think that `dsr` was unused. See line 317:
  for (int dsr=0; ...
* VLP-16: skip badly formatted data packets (`#62 <https://github.com/ros-drivers/velodyne/issues/62>`_, `#63 <https://github.com/ros-drivers/velodyne/issues/63>`_)
* restore VLP-16 min_range setting to 0.4 (`#60 <https://github.com/ros-drivers/velodyne/issues/60>`_)
  NOTE: There is still some other problem keeping that from working.
* permit min_range settings below 0.9 meters (`#60 <https://github.com/ros-drivers/velodyne/issues/60>`_)
  No known models are currently known to return closer measurements.
* Merge pull request `#55 <https://github.com/ros-drivers/velodyne/issues/55>`_ from lemiant/azimuth_bug_VLP16
  Fixed azimuth overflow bug.
* Fixed azimuth overflow bug.
  For interpolated azimuth values between 35999.5 and 36000.0 the nested round(fmod())
  yields a value of 36000 which is invalid and overflows the pre-computed sin/cos arrays,
  since they only go form 0..35999
* Merge pull request `#51 <https://github.com/ros-drivers/velodyne/issues/51>`_ from kunlileo/master
  Added vertical sin angle correction
* Added vertical sin angle correction
* Merge pull request `#47 <https://github.com/ros-drivers/velodyne/issues/47>`_ from prclibo/master
  fixed rounding bug in intensity calculation found by songshiyu
* fixed rounding bug in intensity calculation found by songshiyu
* fix some overly long C++ source lines
* Merge pull request `#44 <https://github.com/ros-drivers/velodyne/issues/44>`_ from SISegwayRmp/master
  adding driver and pointcloud support for the VLP16
* missed the space in the file name which caused the build to fail, removed space before extension
* adding the VLP16 test scripts and updating the CMakeLists to include the test file from http://download.ros.org/data/velodyne/vlp16.pcap
* adding support for the VLP16
* Merge pull request `#43 <https://github.com/ros-drivers/velodyne/issues/43>`_ from prclibo/fix_rawdata
  fixed point computation according to the 64e_s2(.1) velodyne manual
* fixed point computation according to the 64e_s2(.1) velodyne manual, with luopei"s help
* Merge pull request `#41 <https://github.com/ros-drivers/velodyne/issues/41>`_ from prclibo/master
  fixed a calibration file parsing bug
* Merge pull request `#42 <https://github.com/ros-drivers/velodyne/issues/42>`_ from prclibo/fix_gen_calibration
  fixed gen_calibration min/max intensity type
* fixed gen_calibration min/max intensity type
* fixed a calibration file parsing bug
* Contributors: Adam Stambler, Alex Rodrigues, Alexander Schaefer, Andreas Wachaja, Bo Li, Daniel Jartoux, Gabor Meszaros, Jack OQuin, Jose Luis Blanco-Claraco, Joshua Whitley, Kevin Hallenbeck, Kris Kozak, Kun Li, Micho Radovnikovich, Scott K Logan, Thomas Solatges, Todor Stoyanov, William Woodall, jack.oquin, libo24, phussey, piyushk, pomerlef

1.2.0 (2014-08-06)
------------------

* velodyne_pointcloud: remove model-dependent "constants" from
  rawdata.h (`#28
  <https://github.com/ros-drivers/velodyne/issues/28>`_)
* velodyne_pointcloud: change default min_range to 0.9 meters (`#25
  <https://github.com/ros-drivers/velodyne/issues/25>`_)
* Added support for YAML-CPP 0.5+ (`#23
  <https://github.com/ros-drivers/velodyne/pull/23>`_).
* Add dynamic_reconfigure feature.
* Add angular limits to the output point cloud, useful for omitting
  part of it. (`#22 <https://github.com/ros-drivers/velodyne/pull/22>`_).
* Contributors: Jack OQuin, Scott K Logan, Thomas Solatges

1.1.2 (2013-11-05)
------------------

 * Move unit test data to download.ros.org (`#18`_).
 * Install missing gen_calibration.py script (`#20`_).

1.1.1 (2013-07-30)
------------------

 * Fix lost frame_id transform problem caused by PCL 1.7 fix (`#13`_).
 * Add support for HDL-64E S2 and S2.1 models, which were not working
   before (`#11`_), thanks to Gabor Meszaros (`#12`_).
 * Add additional parameters to launch files (`#14`_).
 * Contributors: Gabor Meszaros, Jack OQuin

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

 * Only include "enabled" lasers in YAML calibration file.
 * New param subdirectory for parameter files.
 * Add launch file for the HDL-32E.
 * Add rviz_points.vcg file for viewing Velodyne point clouds with rviz.
 * Fix bug when reading configuration with default minIntensity.
 * Add unit tests with 32E data.
 * Released to Electric, Fuerte and Groovy.

0.9.0 (2012-04-03)
------------------

 * Completely revised API, anticipating a 1.0.0 release.
 * HDL-32E device support.
 * New YAML configuration file format.
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
.. _`#50`: https://github.com/ros-drivers/velodyne/issue/50
