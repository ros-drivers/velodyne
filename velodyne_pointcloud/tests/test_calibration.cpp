//
// C++ unit tests for calibration interface.
//

#include <gtest/gtest.h>

#include <ros/package.h>
#include <velodyne_pointcloud/calibration.h>
using namespace velodyne_pointcloud;

// global test data
std::string g_package_name("velodyne_pointcloud");
std::string g_package_path;

void init_global_data(void)
{
  g_package_path = ros::package::getPath(g_package_name);
}

///////////////////////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////////////////////

TEST(Calibration, missing_file)
{
  Calibration calibration;
  calibration.read("./no_such_file.yaml");
  EXPECT_FALSE(calibration.initialized);
}

TEST(Calibration, vlp16)
{
  Calibration calibration(g_package_path + "/params/VLP16db.yaml");
  EXPECT_TRUE(calibration.initialized);
}

TEST(Calibration, hdl32e)
{
  Calibration calibration(g_package_path + "/params/32db.yaml");
  EXPECT_TRUE(calibration.initialized);
}

TEST(Calibration, hdl64e)
{
  Calibration calibration(g_package_path + "/params/64e_utexas.yaml");
  EXPECT_TRUE(calibration.initialized);
}

TEST(Calibration, hdl64e_s21)
{
  Calibration calibration(g_package_path + "/params/64e_s2.1-sztaki.yaml");
  EXPECT_TRUE(calibration.initialized);
}

TEST(Calibration, hdl64e_s2_float_intensities)
{
  Calibration calibration(g_package_path +
                          "/tests/issue_84_float_intensities.yaml");
  EXPECT_TRUE(calibration.initialized);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  init_global_data();
  return RUN_ALL_TESTS();
}

