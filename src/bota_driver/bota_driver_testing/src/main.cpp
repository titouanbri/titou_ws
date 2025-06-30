/**
 * @authors     Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Main test file
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

using ::testing::InitGoogleTest;

int main(int argc, char** argv)
{
  InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "bota_driver_testing");
  return RUN_ALL_TESTS();
}