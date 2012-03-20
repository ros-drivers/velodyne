# Software License Agreement (BSD License)
#
# Copyright (C) 2012, Austin Robot Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Austin Robot Technology, Inc. nor the names
#    of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

"""
Generate YAML calibration file from Velodyne db.xml.
"""

# this is still under construction...

from __future__ import print_function

import sys
from xml.etree import ElementTree

# TODO: parse the command line
xmlFile = '../tests/HDL-32-db.xml'
yamlFile = '../tests/HDL-32-db.yaml'

db = None
try:
    db = ElementTree.parse(xmlFile)
    if db is None:
        print('failed to parse ' + xmlFile)
        sys.exit(9)
except IOError:
    print('failed to read' + xmlFile)
    sys.exit(9)

# create a dictionary to hold all relevant calibration values
calibration = {'num_lasers': 0, 'lasers': {}, 'pitch': 0.0, 'roll': 0.0}

# add minimum laser intensities
index = 0
for el in db.find('DB/minIntensity_'):
    if el.tag == 'count':
        calibration['num_lasers'] = el.text
    elif el.tag == 'item':
        calibration['lasers'][index] = {'min_intensity': el.text}
        index += 1

# add maximum laser intensities
index = 0
for el in db.find('DB/maxIntensity_'):
    if el.tag == 'count':
        # check count matches expected number of lasers
        if calibration['num_lasers'] != el.text:
            print('invalid number of lasers: ' + el.text)
            parseError = True
    elif el.tag == 'item':
        calibration['lasers'][index]['max_intensity'] = el.text
        index += 1

# add calibration information for each laser
for el in db.find('DB/points_'):
    if el.tag == 'count':
        # check count matches expected number of lasers
        if calibration['num_lasers'] != el.text:
            print('invalid number of lasers: ' + el.text)
            parseError = True
        calibration['num_lasers'] = el.text
    elif el.tag == 'item':
        for px in el:
            for field in px:
                if field.tag == 'id_':
                    index = int(field.text)
                    calibration['lasers'][index]['laser_id'] = index
                elif field.tag == 'rotCorrection_':
                    calibration['lasers'][index]['rot_correction'] = float(field.text)
                elif field.tag == 'vertCorrection_':
                    calibration['lasers'][index]['vert_correction'] = float(field.text)
                elif field.tag == 'distCorrection_':
                    calibration['lasers'][index]['dist_correction'] = float(field.text)
                elif field.tag == 'distCorrectionX_':
                    calibration['lasers'][index]['dist_correction_x'] = float(field.text)
                elif field.tag == 'distCorrectionY_':
                    calibration['lasers'][index]['dist_correction_y'] = float(field.text)
                elif field.tag == 'vertOffsetCorrection_':
                    calibration['lasers'][index]['vert_offset_correction'] = float(field.text)
                elif field.tag == 'horizOffsetCorrection_':
                    calibration['lasers'][index]['horiz_offset_correction'] = float(field.text)
                elif field.tag == 'focalDistance_':
                    calibration['lasers'][index]['focal_distance'] = float(field.text)
                elif field.tag == 'focalSlope_':
                    calibration['lasers'][index]['focal_slope'] = float(field.text)

# debug dump of accumulated calibration data
print(calibration)
