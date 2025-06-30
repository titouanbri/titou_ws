/**
 * @authors     Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Main test file
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

using ::testing::InitGoogleTest;

/* RUN TESTS */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rokubimini_test");
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
