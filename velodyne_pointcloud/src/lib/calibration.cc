/**
 * \file  calibration.cc
 * \brief  
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, Austin Robot Technology, The University of Texas at Austin
 *
 * License: Modified BSD License
 *
 * $ Id: 02/14/2012 11:36:36 AM piyushk $
 */

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <yaml-cpp/yaml.h>

#include <velodyne_pointcloud/calibration.h>

namespace velodyne_pointcloud {

  const std::string NUM_LASERS = "num_lasers";
  const std::string PITCH = "pitch";
  const std::string ROLL = "roll";
  const std::string LASERS = "lasers";
  const std::string LASER_ID = "laser_id";
  const std::string ROT_CORRECTION = "rot_correction";
  const std::string VERT_CORRECTION = "vert_correction";
  const std::string DIST_CORRECTION = "dist_correction";
  const std::string TWO_PT_CORRECTION_AVAILABLE = "two_pt_correction_available";
  const std::string DIST_CORRECTION_X = "dist_correction_x";
  const std::string DIST_CORRECTION_Y = "dist_correction_y";
  const std::string VERT_OFFSET_CORRECTION = "vert_offset_correction";
  const std::string HORIZ_OFFSET_CORRECTION = "vert_offset_correction";
  const std::string MAX_INTENSITY = "max_intensity";
  const std::string MIN_INTENSITY = "min_intensity";
  const std::string FOCAL_DISTANCE = "focal_distance";
  const std::string FOCAL_SLOPE = "focal_slope";

  void operator >> (const YAML::Node& node, std::pair<int, LaserCorrection>& correction) {
    node[LASER_ID] >> correction.first;
    node[ROT_CORRECTION] >> correction.second.rot_correction;
    node[VERT_CORRECTION] >> correction.second.vert_correction;
    node[DIST_CORRECTION] >> correction.second.dist_correction;
    node[TWO_PT_CORRECTION_AVAILABLE] >> correction.second.two_pt_correction_available;
    node[DIST_CORRECTION_X] >> correction.second.dist_correction_x;
    node[DIST_CORRECTION_Y] >> correction.second.dist_correction_y;
    node[VERT_OFFSET_CORRECTION] >> correction.second.vert_offset_correction;
    node[HORIZ_OFFSET_CORRECTION] >> correction.second.horiz_offset_correction;
    node[MAX_INTENSITY] >> correction.second.max_intensity;
    node[MIN_INTENSITY] >> correction.second.min_intensity;
    node[FOCAL_DISTANCE] >> correction.second.focal_distance;
    node[FOCAL_SLOPE] >> correction.second.focal_slope;

    // Calculate cached values
    correction.second.cos_rot_correction = cosf(correction.second.rot_correction);
    correction.second.sin_rot_correction = sinf(correction.second.rot_correction);
    correction.second.cos_vert_correction = cosf(correction.second.vert_correction);
    correction.second.sin_vert_correction = sinf(correction.second.vert_correction);
  }

  void operator >> (const YAML::Node& node, Calibration& calibration) {
    int num_lasers;
    node[NUM_LASERS] >> num_lasers;
    node[PITCH] >> calibration.pitch;
    node[ROLL] >> calibration.roll;
    const YAML::Node& lasers = node[LASERS];
    calibration.laser_corrections.clear();
    for (int i = 0; i < num_lasers; i++) {
      std::pair<int, LaserCorrection> correction;
      lasers[i] >> correction;
      calibration.laser_corrections.insert(correction);     
    }
  }

  YAML::Emitter& operator << (YAML::Emitter& out, const std::pair<int, LaserCorrection> correction) {
    out << YAML::BeginMap;
    out << YAML::Key << LASER_ID << YAML::Value << correction.first;
    out << YAML::Key << ROT_CORRECTION << YAML::Value << correction.second.rot_correction;
    out << YAML::Key << VERT_CORRECTION << YAML::Value << correction.second.vert_correction;
    out << YAML::Key << DIST_CORRECTION << YAML::Value << correction.second.dist_correction;
    out << YAML::Key << TWO_PT_CORRECTION_AVAILABLE << YAML::Value << correction.second.two_pt_correction_available;
    out << YAML::Key << DIST_CORRECTION_X << YAML::Value << correction.second.dist_correction_x;
    out << YAML::Key << DIST_CORRECTION_Y << YAML::Value << correction.second.dist_correction_y;
    out << YAML::Key << VERT_OFFSET_CORRECTION << YAML::Value << correction.second.vert_offset_correction;
    out << YAML::Key << HORIZ_OFFSET_CORRECTION << YAML::Value << correction.second.horiz_offset_correction;
    out << YAML::Key << MAX_INTENSITY << YAML::Value << correction.second.max_intensity;
    out << YAML::Key << MIN_INTENSITY << YAML::Value << correction.second.min_intensity;
    out << YAML::Key << FOCAL_DISTANCE << YAML::Value << correction.second.focal_distance;
    out << YAML::Key << FOCAL_SLOPE << YAML::Value << correction.second.focal_slope;
    out << YAML::EndMap;
    return out;
  }

  YAML::Emitter& operator << (YAML::Emitter& out, const Calibration& calibration) {
    out << YAML::BeginMap;
    out << YAML::Key << NUM_LASERS << YAML::Value << calibration.laser_corrections.size();
    out << YAML::Key << PITCH << YAML::Value << calibration.pitch;
    out << YAML::Key << ROLL << YAML::Value << calibration.roll;
    out << YAML::Key << LASERS << YAML::Value << YAML::BeginSeq;
    for (std::map<int, LaserCorrection>::const_iterator it = calibration.laser_corrections.begin();
         it != calibration.laser_corrections.end(); it++) {
      out << *it; 
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
    return out;
  }

  void Calibration::read(const std::string& calibration_file) {
    std::ifstream fin(calibration_file.c_str());
    if (!fin.is_open()) {
      initialized = false;
      return;
    }
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);
    doc >> *this;
    fin.close();
    initialized = true;
  }

  void Calibration::write(const std::string& calibration_file) {
    std::ofstream fout(calibration_file.c_str());
    YAML::Emitter out;
    out << *this;
    fout << out.c_str();
    fout.close();
  }
  
} /* velodyne_pointcloud */
