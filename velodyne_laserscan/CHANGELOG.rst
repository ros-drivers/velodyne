^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package velodyne_laserscan
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.0 (2023-05-27)
------------------
* using std::round to calculate the scan array size (`#459 <https://github.com/ros-drivers/velodyne/issues/459>`_)
* Contributors: Gianluca Bardaro

2.3.0 (2022-07-08)
------------------
* Updating maintainer email address. (`#450 <https://github.com/ros-drivers/velodyne/issues/450>`_)
  * Updating maintainer email address.
  * chore: update maintainer email address
  Co-authored-by: Joshua Whitley <jwhitley@autonomoustuff.com>
* Fix small warnings from build and test.
* Add laserscan support for new PointXYZIR structure (`#316 <https://github.com/ros-drivers/velodyne/issues/316>`_) (`#439 <https://github.com/ros-drivers/velodyne/issues/439>`_)
  * Add laserscan support for new PointXYZIR structure
  * Support PointCloud2 offsets that are multiples of 4
  Co-authored-by: Kevin Hallenbeck <khallenbeck@dataspeedinc.com>
* Replace deprecated argument names in launch (`#430 <https://github.com/ros-drivers/velodyne/issues/430>`_)
* 2.1.1
* Updating for first Galactic release
* Contributors: Chris Lalancette, Daisuke Nishimatsu, Joshua Whitley, Keane Quigley

2.1.1 (2021-08-23)
------------------

2.1.0 (2020-07-10)
------------------
* Fixing Foxy-specific uncrustify errors.
* Contributors: Joshua Whitley

2.0.0 (2020-07-10)
------------------
* Fix dependencies in package.xml (`#331 <https://github.com/ros-drivers/velodyne/issues/331>`_)
  Ensure that we depend on ament_cmake_ros as appropriate.
* Fix ring retrieval from float pointcloud iterator. (`#322 <https://github.com/ros-drivers/velodyne/issues/322>`_)
  * Fix ring retrieval from float pointcloud iterator.
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
* ROS2: Add Linters to velodyne_laserscan (`#303 <https://github.com/ros-drivers/velodyne/issues/303>`_)
  * Adding tests and updating file naming.
  * Fixing copyright and xml issues.
  * Fixing linter stuff on velodyne_laserscan.
  * Fixing typo in conversion.
* Switch the style on scope-constants to google style.
* Add in example launch files.
* Disable copy, move, and assign operators.
* Build the components so the velodyne can be run as a component.
* Add in NodeOptions to node constructors.
* Make sure to install the header files for laserscan and pointcloud.
* Switch to RCLCPP_WARN_ONCE.
* Make sure to only print the "slower generic method" message once.
* Add explicit, final, and override to classes where appropriate.
* Merge pull request `#251 <https://github.com/ros-drivers/velodyne/issues/251>`_ from clalancette/dashing-devel3
  ROS 2 Dashing port
* Merge pull request `#234 <https://github.com/ros-drivers/velodyne/issues/234>`_ from kmhallen/c++11
  Set minimum C++ standard to C++11
* Merge pull request `#231 <https://github.com/ros-drivers/velodyne/issues/231>`_ from ros-drivers/ci/test_better_output
  CI: Adding roslint as separate step. Limiting output of catkin build.
* Merge pull request `#227 <https://github.com/ros-drivers/velodyne/issues/227>`_ from ros-drivers/roslint
  Applying roslint to velodyne_pointcloud.
* Merge pull request `#226 <https://github.com/ros-drivers/velodyne/issues/226>`_ from ros-drivers/roslint
  Adding roslint to velodyne_laserscan
* Contributors: Chris Lalancette, Joshua Whitley, Kevin Hallenbeck, Matthew Pitropov, Michel Hidalgo, Sebastian, Sebastian Pütz

1.5.2 (2019-01-28)
------------------

1.5.1 (2018-12-10)
------------------

1.5.0 (2018-10-19)
------------------

1.4.0 (2018-09-19)
------------------
* Merge pull request `#170 <https://github.com/ros-drivers/velodyne/issues/170>`_ from ros-drivers/maint/move_header_files
* Moving header files to traditional location inside include folders.
* Merge pull request `#160 <https://github.com/ros-drivers/velodyne/issues/160>`_ from ros-drivers/maint/updating_package_xml_to_v2
* Updated all package.xmls to ver 2. Cleaned up catkin_lint errors.
  All package.xml files are now compatible with version 2 of the
  package.xml specification in REP 140. Removed some unnecessary
  execute permissions on a few files. Fixed a missing test_depend.
* Merge pull request `#146 <https://github.com/ros-drivers/velodyne/issues/146>`_ from stsundermann/patch-2
  Use std::abs instead of fabsf
* Merge pull request `#150 <https://github.com/ros-drivers/velodyne/issues/150>`_ from ros-drivers/mikaelarguedas-patch-1
* update to use non deprecated pluginlib macro
* Use std::abs instead of fabsf
  cfg\_.resolution is double but fabsf takes a float which may cause truncation of value.
* Contributors: Andre Volk, CNR, Joshua Whitley, Mikael Arguedas, Stephan Sundermann

1.3.0 (2017-11-10)
------------------
* Merge pull request `#110 <https://github.com/ros-drivers/velodyne/issues/110>`_ from kmhallen/master
  Added velodyne_laserscan package
* Added tests for velodyne_laserscan
* Fixed validating PointCloud2 field types
* Package.xml format version 2
* Merge pull request `#1 <https://github.com/ros-drivers/velodyne/issues/1>`_ from volkandre/master
  Fixed bug. Laserscans now cover full 360 degrees.
* Fixed bug. Laserscans now cover full 360 degrees.
* Added velodyne_laserscan package and inserted into existing launch files
* Contributors: Joshua Whitley, Kevin Hallenbeck, kmhallen, volkandre
