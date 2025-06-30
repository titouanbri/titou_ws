/**
 * @authors     Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Tests Command
 */

#include <gtest/gtest.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Wrench.h>
#include <ros/ros.h>

namespace bota_driver_testing
{
class BotaDriverTestFTOffsets : public ::testing::Test
{
protected:
  ros::Subscriber sub_;
  ros::NodeHandle nh_{ "~" };
  std::uint32_t msgCount_;
  geometry_msgs::Wrench meanWrenchOffset_;
  std::string topicName_;
  float testDuration_;

  BotaDriverTestFTOffsets() : msgCount_(0)
  {
    meanWrenchOffset_.force.x = 0;
    meanWrenchOffset_.force.y = 0;
    meanWrenchOffset_.force.z = 0;
    meanWrenchOffset_.torque.x = 0;
    meanWrenchOffset_.torque.y = 0;
    meanWrenchOffset_.torque.z = 0;
  }

  ~BotaDriverTestFTOffsets() override
  {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  void SetUp() override
  {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  void TearDown() override
  {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

public:
  void meanwrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    msgCount_++;
    meanWrenchOffset_.force.x += ((msg->wrench.force.x - meanWrenchOffset_.force.x) / msgCount_);
    meanWrenchOffset_.force.y += ((msg->wrench.force.y - meanWrenchOffset_.force.y) / msgCount_);
    meanWrenchOffset_.force.z += ((msg->wrench.force.z - meanWrenchOffset_.force.z) / msgCount_);
    meanWrenchOffset_.torque.x += ((msg->wrench.torque.x - meanWrenchOffset_.torque.x) / msgCount_);
    meanWrenchOffset_.torque.y += ((msg->wrench.torque.y - meanWrenchOffset_.torque.y) / msgCount_);
    meanWrenchOffset_.torque.z += ((msg->wrench.torque.z - meanWrenchOffset_.torque.z) / msgCount_);
  }
};

TEST_F(BotaDriverTestFTOffsets, SetFTOffsets)
{
  SCOPED_TRACE("SetFTOffsets");
  ros::Time time_offset;
  ASSERT_EQ(nh_.getParam("topic_name", topicName_), true);
  sub_ = nh_.subscribe(topicName_, 1000, &BotaDriverTestFTOffsets::meanwrenchCallback, (BotaDriverTestFTOffsets*)this);
  ASSERT_EQ(nh_.getParam("test_duration", testDuration_), true);
  time_offset = ros::Time::now() + ros::Duration(testDuration_);
  while (ros::ok() && ros::Time::now() < time_offset)
  {
    ros::spinOnce();
    if (HasFatalFailure())
    {
      FAIL() << "Fatal errors occurred.\n";
      return;
    }
  }
  EXPECT_GT(msgCount_, 0U);
  ASSERT_NEAR(meanWrenchOffset_.force.x, 10000.0, 1000);
  ASSERT_NEAR(meanWrenchOffset_.force.y, -10000.0, 1000);
  ASSERT_NEAR(meanWrenchOffset_.force.z, 20000.0, 1000);
  ASSERT_NEAR(meanWrenchOffset_.torque.x, -20000.0, 100);
  ASSERT_NEAR(meanWrenchOffset_.torque.y, 30000.0, 100);
  ASSERT_NEAR(meanWrenchOffset_.torque.z, -30000.0, 100);
}

}  // namespace bota_driver_testing