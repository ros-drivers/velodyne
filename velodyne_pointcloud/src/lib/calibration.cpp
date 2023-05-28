// Copyright 2012, 2019 Austin Robot Technology, Piyush Khandelwal, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "velodyne_pointcloud/calibration.hpp"

#ifdef HAVE_NEW_YAMLCPP

namespace YAML
{

// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator>>(const YAML::Node & node, T & i)
{
  i = node.as<T>();
}

}  // namespace YAML

#endif  // HAVE_NEW_YAMLCPP

namespace velodyne_pointcloud
{

constexpr char NUM_LASERS[] = "num_lasers";
constexpr char DISTANCE_RESOLUTION[] = "distance_resolution";
constexpr char LASERS[] = "lasers";
constexpr char LASER_ID[] = "laser_id";
constexpr char ROT_CORRECTION[] = "rot_correction";
constexpr char VERT_CORRECTION[] = "vert_correction";
constexpr char DIST_CORRECTION[] = "dist_correction";
constexpr char TWO_PT_CORRECTION_AVAILABLE[] =
  "two_pt_correction_available";
constexpr char DIST_CORRECTION_X[] = "dist_correction_x";
constexpr char DIST_CORRECTION_Y[] = "dist_correction_y";
constexpr char VERT_OFFSET_CORRECTION[] = "vert_offset_correction";
constexpr char HORIZ_OFFSET_CORRECTION[] = "horiz_offset_correction";
constexpr char MAX_INTENSITY[] = "max_intensity";
constexpr char MIN_INTENSITY[] = "min_intensity";
constexpr char FOCAL_DISTANCE[] = "focal_distance";
constexpr char FOCAL_SLOPE[] = "focal_slope";

/** Read calibration for a single laser. */
void operator>>(const YAML::Node & node, std::pair<int, LaserCorrection> & correction)
{
  node[LASER_ID] >> correction.first;
  node[ROT_CORRECTION] >> correction.second.rot_correction;
  node[VERT_CORRECTION] >> correction.second.vert_correction;
  node[DIST_CORRECTION] >> correction.second.dist_correction;

#ifdef HAVE_NEW_YAMLCPP

  if (node[TWO_PT_CORRECTION_AVAILABLE]) {
    node[TWO_PT_CORRECTION_AVAILABLE] >>
    correction.second.two_pt_correction_available;
  } else {
#else

  if (const YAML::Node * pName = node.FindValue(TWO_PT_CORRECTION_AVAILABLE)) {
    *pName >> correction.second.two_pt_correction_available;
  } else {
#endif
    correction.second.two_pt_correction_available = false;
  }

  node[DIST_CORRECTION_X] >> correction.second.dist_correction_x;
  node[DIST_CORRECTION_Y] >> correction.second.dist_correction_y;
  node[VERT_OFFSET_CORRECTION] >> correction.second.vert_offset_correction;

#ifdef HAVE_NEW_YAMLCPP

  if (node[HORIZ_OFFSET_CORRECTION]) {
    node[HORIZ_OFFSET_CORRECTION] >>
    correction.second.horiz_offset_correction;
  } else {
#else

  if (const YAML::Node * pName = node.FindValue(HORIZ_OFFSET_CORRECTION)) {
    *pName >> correction.second.horiz_offset_correction;
  } else {
#endif
    correction.second.horiz_offset_correction = 0;
  }

  float max_intensity_float = 255.0;

#ifdef HAVE_NEW_YAMLCPP

  if (node[MAX_INTENSITY]) {
    node[MAX_INTENSITY] >> max_intensity_float;
  }

#else

  if (const YAML::Node * pName = node.FindValue(MAX_INTENSITY)) {
    *pName >> max_intensity_float;
  }

#endif

  correction.second.max_intensity = ::floorf(max_intensity_float);

  float min_intensity_float = 0.0;

#ifdef HAVE_NEW_YAMLCPP

  if (node[MIN_INTENSITY]) {
    node[MIN_INTENSITY] >> min_intensity_float;
  }

#else

  if (const YAML::Node * pName = node.FindValue(MIN_INTENSITY)) {
    *pName >> min_intensity_float;
  }

#endif

  correction.second.min_intensity = ::floorf(min_intensity_float);

  node[FOCAL_DISTANCE] >> correction.second.focal_distance;
  node[FOCAL_SLOPE] >> correction.second.focal_slope;

  // Calculate cached values
  correction.second.cos_rot_correction =
    ::cosf(correction.second.rot_correction);
  correction.second.sin_rot_correction =
    ::sinf(correction.second.rot_correction);
  correction.second.cos_vert_correction =
    ::cosf(correction.second.vert_correction);
  correction.second.sin_vert_correction =
    ::sinf(correction.second.vert_correction);

  correction.second.laser_ring = 0;   // clear initially (set later)
}

/** Read entire calibration file. */
void operator>>(const YAML::Node & node, Calibration & calibration)
{
  int num_lasers;
  node[NUM_LASERS] >> num_lasers;
  float distance_resolution_m;
  node[DISTANCE_RESOLUTION] >> distance_resolution_m;
  const YAML::Node & lasers = node[LASERS];
  calibration.laser_corrections.clear();
  calibration.num_lasers = num_lasers;
  calibration.distance_resolution_m = distance_resolution_m;
  calibration.laser_corrections.resize(num_lasers);

  for (int i = 0; i < num_lasers; i++) {
    std::pair<int, LaserCorrection> correction;
    lasers[i] >> correction;
    const int index = correction.first;

    if (static_cast<size_t>(index) >= calibration.laser_corrections.size()) {
      calibration.laser_corrections.resize(index + 1);
    }

    calibration.laser_corrections[index] = (correction.second);
  }

  // For each laser ring, find the next-smallest vertical angle.
  //
  // This implementation is simple, but not efficient.  That is OK,
  // since it only runs while starting up.
  double next_angle = -std::numeric_limits<double>::infinity();

  for (int ring = 0; ring < num_lasers; ++ring) {
    // find minimum remaining vertical offset correction
    double min_seen = std::numeric_limits<double>::infinity();
    int next_index = num_lasers;

    for (int j = 0; j < num_lasers; ++j) {
      double angle = calibration.laser_corrections[j].vert_correction;

      if (next_angle < angle && angle < min_seen) {
        min_seen = angle;
        next_index = j;
      }
    }

    if (next_index < num_lasers) {  // anything found in this ring?
      // store this ring number with its corresponding laser number
      calibration.laser_corrections[next_index].laser_ring = ring;
      next_angle = min_seen;
    }
  }
}

Calibration::Calibration(const std::string & calibration_file)
: distance_resolution_m(0.002f)
{
  std::ifstream fin(calibration_file.c_str());

  if (!fin.is_open()) {
    throw std::runtime_error("Failed to open calibration file");
  }

  try {
    YAML::Node doc;

#ifdef HAVE_NEW_YAMLCPP

    fin.close();
    doc = YAML::LoadFile(calibration_file);

#else

    YAML::Parser parser(fin);
    parser.GetNextDocument(doc);

#endif
    doc >> *this;
  } catch (YAML::Exception & e) {
    fin.close();
    throw std::runtime_error("YAML Exception: " + std::string(e.what()));
  }

  fin.close();
}

}  // namespace velodyne_pointcloud
