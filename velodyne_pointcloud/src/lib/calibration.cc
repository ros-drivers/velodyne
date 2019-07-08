/**
 * \file  calibration.cc
 * \brief
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, Austin Robot Technology,
 *                     The University of Texas at Austin
 *
 * License: Modified BSD License
 *
 * $ Id: 02/14/2012 11:36:36 AM piyushk $
 */

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <utility>

#include <yaml-cpp/yaml.h>

#ifdef HAVE_NEW_YAMLCPP
namespace YAML
{
  // The >> operator disappeared in yaml-cpp 0.5, so this function is
  // added to provide support for code written under the yaml-cpp 0.3 API.
  template<typename T>
  void operator >> (const YAML::Node& node, T& i)
  {
    i = node.as<T>();
  }
} /* YAML */
#endif // HAVE_NEW_YAMLCPP

#include <rclcpp/rclcpp.hpp>
#include <velodyne_pointcloud/calibration.h>

namespace velodyne_pointcloud
{
  const std::string NUM_LASERS = "num_lasers";
  const std::string DISTANCE_RESOLUTION = "distance_resolution";
  const std::string LASERS = "lasers";
  const std::string LASER_ID = "laser_id";
  const std::string ROT_CORRECTION = "rot_correction";
  const std::string VERT_CORRECTION = "vert_correction";
  const std::string DIST_CORRECTION = "dist_correction";
  const std::string TWO_PT_CORRECTION_AVAILABLE =
    "two_pt_correction_available";
  const std::string DIST_CORRECTION_X = "dist_correction_x";
  const std::string DIST_CORRECTION_Y = "dist_correction_y";
  const std::string VERT_OFFSET_CORRECTION = "vert_offset_correction";
  const std::string HORIZ_OFFSET_CORRECTION = "horiz_offset_correction";
  const std::string MAX_INTENSITY = "max_intensity";
  const std::string MIN_INTENSITY = "min_intensity";
  const std::string FOCAL_DISTANCE = "focal_distance";
  const std::string FOCAL_SLOPE = "focal_slope";

  /** Read calibration for a single laser. */
  void operator >> (const YAML::Node& node,
                    std::pair<int, LaserCorrection>& correction)
  {
    node[LASER_ID] >> correction.first;
    node[ROT_CORRECTION] >> correction.second.rot_correction;
    node[VERT_CORRECTION] >> correction.second.vert_correction;
    node[DIST_CORRECTION] >> correction.second.dist_correction;
#ifdef HAVE_NEW_YAMLCPP
    if (node[TWO_PT_CORRECTION_AVAILABLE])
      {
        node[TWO_PT_CORRECTION_AVAILABLE] >>
          correction.second.two_pt_correction_available;
      }
#else
    if (const YAML::Node *pName = node.FindValue(TWO_PT_CORRECTION_AVAILABLE))
      {
        *pName >> correction.second.two_pt_correction_available;
      }
#endif
    else
      {
        correction.second.two_pt_correction_available = false;
      }
    node[DIST_CORRECTION_X] >> correction.second.dist_correction_x;
    node[DIST_CORRECTION_Y] >> correction.second.dist_correction_y;
    node[VERT_OFFSET_CORRECTION] >> correction.second.vert_offset_correction;
#ifdef HAVE_NEW_YAMLCPP
    if (node[HORIZ_OFFSET_CORRECTION])
      {
        node[HORIZ_OFFSET_CORRECTION] >>
          correction.second.horiz_offset_correction;
      }
#else
    if (const YAML::Node *pName = node.FindValue(HORIZ_OFFSET_CORRECTION))
      {
        *pName >> correction.second.horiz_offset_correction;
      }
#endif
    else
      {
        correction.second.horiz_offset_correction = 0;
      }

    float max_intensity_float = 255.0;
#ifdef HAVE_NEW_YAMLCPP
    if (node[MAX_INTENSITY])
      {
        node[MAX_INTENSITY] >> max_intensity_float;
      }
#else
    if (const YAML::Node *pName = node.FindValue(MAX_INTENSITY))
      {
        *pName >> max_intensity_float;
      }
#endif
    correction.second.max_intensity = ::floor(max_intensity_float);

    float min_intensity_float = 0.0;
#ifdef HAVE_NEW_YAMLCPP
    if (node[MIN_INTENSITY])
      {
        node[MIN_INTENSITY] >> min_intensity_float;
      }
#else
    if (const YAML::Node *pName = node.FindValue(MIN_INTENSITY))
      {
        *pName >> min_intensity_float;
      }
#endif
    correction.second.min_intensity = ::floor(min_intensity_float);

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
  void operator >> (const YAML::Node& node, Calibration& calibration)
  {
    int num_lasers;
    node[NUM_LASERS] >> num_lasers;
    float distance_resolution_m;
    node[DISTANCE_RESOLUTION] >> distance_resolution_m;
    const YAML::Node& lasers = node[LASERS];
    calibration.laser_corrections.clear();
    calibration.num_lasers = num_lasers;
    calibration.distance_resolution_m = distance_resolution_m;
    calibration.laser_corrections.resize(num_lasers);
    for (int i = 0; i < num_lasers; i++)
      {
        std::pair<int, LaserCorrection> correction;
        lasers[i] >> correction;
        const int index = correction.first;
        if (static_cast<size_t>(index) >= calibration.laser_corrections.size())
          {
            calibration.laser_corrections.resize(index+1);
          }
        calibration.laser_corrections[index] = (correction.second);
        calibration.laser_corrections_map.insert(correction);
      }

    // For each laser ring, find the next-smallest vertical angle.
    //
    // This implementation is simple, but not efficient.  That is OK,
    // since it only runs while starting up.
    double next_angle = -std::numeric_limits<double>::infinity();
    for (int ring = 0; ring < num_lasers; ++ring)
      {
        // find minimum remaining vertical offset correction
        double min_seen = std::numeric_limits<double>::infinity();
        int next_index = num_lasers;
        for (int j = 0; j < num_lasers; ++j)
          {
            double angle = calibration.laser_corrections[j].vert_correction;
            if (next_angle < angle && angle < min_seen)
              {
                min_seen = angle;
                next_index = j;
              }
          }

        if (next_index < num_lasers)  // anything found in this ring?
          {

            // store this ring number with its corresponding laser number
            calibration.laser_corrections[next_index].laser_ring = ring;
            next_angle = min_seen;
          }
      }
  }

  void Calibration::read(const std::string& calibration_file)
  {
    std::ifstream fin(calibration_file.c_str());
    if (!fin.is_open())
      {
        initialized = false;
        return;
      }
    initialized = true;
    try
      {
        YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
        fin.close();
        doc = YAML::LoadFile(calibration_file);
#else
        YAML::Parser parser(fin);
        parser.GetNextDocument(doc);
#endif
        doc >> *this;
      }
    catch (YAML::Exception &e)
      {
        std::cerr << "YAML Exception: " << e.what() << std::endl;
        initialized = false;
      }
    fin.close();
  }

}  // namespace velodyne_pointcloud
