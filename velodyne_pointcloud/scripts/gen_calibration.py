#!/usr/bin/python3

# Copyright 2012, 2019 Austin Robot Technology, Joshua Whitley
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
# * Neither the name of {copyright_holder} nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
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

"""
Generate YAML calibration file from Velodyne db.xml.

The input data provided by the manufacturer are in degrees and
centimeters. The YAML file uses radians and meters, following ROS
standards [REP-0103].

"""

import math
from optparse import OptionParser
import os
import sys
from xml.etree import ElementTree

import yaml


class GenCalibration:

    def __init__(self):
        self.calibration_good = True

        self.parse_cmd_args()

        if self.calibration_good:
            print('converting "' + self.xml_file + '" to "' + self.yaml_file + '"')
            self.parse_xml()

        if self.calibration_good:
            self.write_calibration_file()

    def xml_error(self, msg):
        """Handle XML calibration error."""
        self.calibration_good = False
        print('gen_calibration.py: ' + msg)

    def add_laser_calibration(self, laser_num, key, val):
        """Define key and corresponding value for laser_num."""
        if laser_num < len(self.calibration['lasers']):
            self.calibration['lasers'][laser_num][key] = val
        else:
            self.calibration['lasers'].append({key: val})

    def parse_cmd_args(self):
        # parse the command line
        usage = """usage: %prog infile.xml [outfile.yaml]

               Default output file is input file with .yaml suffix."""
        parser = OptionParser(usage=usage)
        options, args = parser.parse_args()

        if len(args) < 1:
            parser.error('XML file name missing')
            sys.exit(9)

        self.xml_file = args[0]

        if len(args) >= 2:
            self.yaml_file = args[1]
        else:
            self.yaml_file, ext = os.path.splitext(self.xml_file)
            self.yaml_file += '.yaml'

            self.calibrationGood = True

    def parse_xml(self):
        db = None

        try:
            db = ElementTree.parse(self.xml_file)
        except IOError:
            self.xml_error('unable to read ' + self.xml_file)
        except ElementTree.ParseError:
            self.xml_error('XML parse failed for ' + self.xml_file)

        if not self.calibration_good:
            sys.exit(2)

        # create a dictionary to hold all relevant calibration values
        self.calibration = {'num_lasers': 0, 'lasers': [], 'distance_resolution': 0.2}
        cm2meters = 0.01   # convert centimeters to meters

        # add enabled flags
        num_enabled = 0
        enabled_lasers = []
        enabled = db.find('DB/enabled_')

        if enabled is None:
            print('no enabled tags found: assuming all 64 enabled')
            num_enabled = 64
            enabled_lasers = [True for i in range(num_enabled)]
        else:
            index = 0
            for el in enabled:
                if el.tag == 'item':
                    this_enabled = int(el.text) != 0
                    enabled_lasers.append(this_enabled)
                    index += 1
                    if this_enabled:
                        num_enabled += 1

        self.calibration['num_lasers'] = num_enabled
        print(str(num_enabled) + ' lasers')

        # add distance resolution (cm)
        distLSB = db.find('DB/distLSB_')

        if distLSB is not None:
            self.calibration['distance_resolution'] = float(distLSB.text) * cm2meters

        # add minimum laser intensities
        minIntensities = db.find('DB/minIntensity_')

        if minIntensities is not None:
            index = 0
            for el in minIntensities:
                if el.tag == 'item':
                    if enabled_lasers[index]:
                        value = int(el.text)
                        if value != 0:
                            self.add_laser_calibration(index, 'min_intensity', value)
                    index += 1

        # add maximum laser intensities
        maxIntensities = db.find('DB/maxIntensity_')

        if maxIntensities is not None:
            index = 0
            for el in maxIntensities:
                if el.tag == 'item':
                    if enabled_lasers[index]:
                        value = int(el.text)
                        if value != 255:
                            self.add_laser_calibration(index, 'max_intensity', value)
                        index += 1

        # add calibration information for each laser
        for el in db.find('DB/points_'):
            if el.tag == 'item':
                for px in el:
                    for field in px:
                        if field.tag == 'id_':
                            index = int(field.text)
                            if not enabled_lasers[index]:
                                break   # skip this laser, it is not enabled
                            self.add_laser_calibration(index, 'laser_id', index)

                        if field.tag == 'rotCorrection_':
                            self.add_laser_calibration(index, 'rot_correction',
                                                       math.radians(float(field.text)))
                        elif field.tag == 'vertCorrection_':
                            self.add_laser_calibration(index, 'vert_correction',
                                                       math.radians(float(field.text)))
                        elif field.tag == 'distCorrection_':
                            self.add_laser_calibration(index, 'dist_correction',
                                                       float(field.text) * cm2meters)
                        elif field.tag == 'distCorrectionX_':
                            self.add_laser_calibration(index, 'dist_correction_x',
                                                       float(field.text) * cm2meters)
                        elif field.tag == 'distCorrectionY_':
                            self.add_laser_calibration(index, 'dist_correction_y',
                                                       float(field.text) * cm2meters)
                        elif field.tag == 'vertOffsetCorrection_':
                            self.add_laser_calibration(index, 'vert_offset_correction',
                                                       float(field.text) * cm2meters)
                        elif field.tag == 'horizOffsetCorrection_':
                            self.add_laser_calibration(index, 'horiz_offset_correction',
                                                       float(field.text) * cm2meters)
                        elif field.tag == 'focalDistance_':
                            self.add_laser_calibration(index, 'focal_distance',
                                                       float(field.text) * cm2meters)
                        elif field.tag == 'focalSlope_':
                            self.add_laser_calibration(index, 'focal_slope', float(field.text))

        # validate input data
        if self.calibration['num_lasers'] <= 0:
            self.xml_error('no lasers defined')
        elif self.calibration['num_lasers'] != num_enabled:
            self.xml_error('inconsistent number of lasers defined')

        # TODO: make sure all required fields are present.
        # (Which ones are required?)

    def write_calibration_file(self):
        # write calibration data to YAML file
        f = open(self.yaml_file, 'w')
        try:
            yaml.dump(self.calibration, f)
        finally:
            f.close()


if __name__ == '__main__':
    GenCalibration()
