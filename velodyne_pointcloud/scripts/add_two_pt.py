#!/usr/bin/python
"""
usage: add_two_pt.py < calibration.yaml > calibration_two_pt.yaml

In order to take into acount *HDL-64* correction_{x,y}, related to 2012 merge:
https://github.com/ros-drivers/velodyne/commit/f30d68735c47312aa73d29203ddb16abc01357f4

https://github.com/ros-drivers/velodyne/blob/master/velodyne_pointcloud/src/lib/rawdata.cc#L438
https://github.com/ros-drivers/velodyne/blob/master/velodyne_pointcloud/src/lib/calibration.cc#L70
"""
import sys
import yaml

calibration = yaml.safe_load(sys.stdin)
for laser in calibration['lasers']:
    laser['two_pt_correction_available'] = True

print(yaml.safe_dump(calibration))
