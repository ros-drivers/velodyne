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

# create a dictionary to hold all the calibration values of each of the lasers
calibration = {'lasers': {}}

# this dictionary holds all the calibration values of each laser
# TODO: add to calibration.lasers[index]
numLasers = 64
lasers = [{} for i in range(numLasers)]

index = 0
for el in db.find('DB/minIntensity_'):
    if el.tag == 'item':
        lasers[index]['min_intensity'] = el.text
        index += 1

index = 0
for el in db.find('DB/maxIntensity_'):
    if el.tag == 'item':
        lasers[index]['max_intensity'] = el.text
        index += 1

for el in db.find('DB/points_'):
    if el.tag == 'count':
        if numLasers != int(el.text):
            print('invalid number of lasers: ' + el.text)
            parseError = True
    elif el.tag == 'item':
        for px in el:
            for field in px:
                if field.tag == 'id_':
                    index = int(field.text)
                    lasers[index]['laser_id'] = field.text
                elif field.tag == 'rotCorrection_':
                    lasers[index]['rot_correction'] = field.text
                elif field.tag == 'vertCorrection_':
                    lasers[index]['vert_correction'] = field.text
                elif field.tag == 'distCorrection_':
                    lasers[index]['dist_correction'] = field.text
                elif field.tag == 'distCorrectionX_':
                    lasers[index]['dist_correction_x'] = field.text
                elif field.tag == 'distCorrectionY_':
                    lasers[index]['dist_correction_y'] = field.text
                elif field.tag == 'vertOffsetCorrection_':
                    lasers[index]['vert_offset_correction'] = field.text
                elif field.tag == 'horizOffsetCorrection_':
                    lasers[index]['horiz_offset_correction'] = field.text
                elif field.tag == 'focalDistance_':
                    lasers[index]['focal_distance'] = field.text
                elif field.tag == 'focalSlope_':
                    lasers[index]['focal_slope'] = field.text

#for item in lasers:
#    for field in item:
#        print(field + ': ' + item[field])
