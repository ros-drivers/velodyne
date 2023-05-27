Change history
==============

2.4.0 (2023-05-27)
------------------
* Unify tf frame parameters between transform and cloud nodes (`#344 <https://github.com/ros-drivers/velodyne/issues/344>`_) (`#453 <https://github.com/ros-drivers/velodyne/issues/453>`_)
  * Unify tf frame parameters between transform and cloud nodes
  At this point there is no need any more for cloud node because transform node includes all features of cloud node.
  Co-authored-by: AndreasR30 <andreas-reich@live.de>
  Co-authored-by: anre <andreas.reich@unibw.de>
* Contributors: Daisuke Nishimatsu

2.3.0 (2022-07-08)
------------------
* Updating maintainer email address. (`#450 <https://github.com/ros-drivers/velodyne/issues/450>`_)
  * Updating maintainer email address.
  * chore: update maintainer email address
  Co-authored-by: Joshua Whitley <jwhitley@autonomoustuff.com>
* Replace deprecated argument names in launch (`#430 <https://github.com/ros-drivers/velodyne/issues/430>`_)
* 2.1.1
* Updating for first Galactic release
* Contributors: Daisuke Nishimatsu, Joshua Whitley, Keane Quigley

2.1.1 (2021-08-23)
------------------

2.1.0 (2020-07-10)
------------------

2.0.0 (2020-07-10)
------------------
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
* Add in example launch files.
* Merge pull request `#251 <https://github.com/ros-drivers/velodyne/issues/251>`_ from clalancette/dashing-devel3
  ROS 2 Dashing port
* Contributors: Andreas Klintberg, Chris Lalancette, Joshua Whitley, Sebastian Pütz

1.5.2 (2019-01-28)
------------------

1.5.1 (2018-12-10)
------------------

1.5.0 (2018-10-19)
------------------

1.4.0 (2018-09-19)
------------------
* Merge pull request `#160 <https://github.com/ros-drivers/velodyne/issues/160>`_ from ros-drivers/maint/updating_package_xml_to_v2
* Updated all package.xmls to ver 2. Cleaned up catkin_lint errors.
  All package.xml files are now compatible with version 2 of the
  package.xml specification in REP 140. Removed some unnecessary
  execute permissions on a few files. Fixed a missing test_depend.
* Contributors: Andre Volk, Joshua Whitley

1.3.0 (2017-11-10)
------------------
* Merge pull request `#110 <https://github.com/ros-drivers/velodyne/issues/110>`_ from kmhallen/master
  Added velodyne_laserscan package
* Added velodyne_laserscan package and inserted into existing launch files
* Contributors: Jack O'Quin, Joshua Whitley, Kevin Hallenbeck

1.2.0 (2014-08-06)
------------------

1.1.2 (2013-11-05)
-------------------

1.1.1 (2013-07-30)
------------------

1.1.0 (2013-07-16)
------------------

1.0.1 (2013-06-15)
------------------

1.0.0 (2013-06-14)
------------------

 * Convert to catkin (`#1`_).
 * Release to Hydro.

0.9.2 (2013-07-08)
------------------

0.9.1 (2012-06-05)
------------------

0.9.0 (2012-04-03)
------------------

 * Completely revised API, anticipating a 1.0.0 release.
 * New velodyne_driver and velodyne_pointcloud packages.
 * Old velodyne_common and velodyne_pcl packages no longer included.
 * Released to Electric, Fuerte and Groovy.

0.2.6 (2011-02-23)
------------------

0.2.5 (2010-11-19)
------------------

 * Initial implementation of new 0.3 interfaces.

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
