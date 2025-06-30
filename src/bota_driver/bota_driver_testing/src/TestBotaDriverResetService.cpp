/**
 * @authors     Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Tests Command
 */

#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Wrench.h>
#include <ros/ros.h>
#include <rokubimini_msgs/ResetWrench.h>

namespace bota_driver_testing
{
class BotaDriverTestResetService : public ::testing::Test
{
protected:
  ros::Subscriber sub_;
  ros::NodeHandle nh_{ "~" };
  std::uint32_t msgCount_;
  geometry_msgs::Wrench meanWrenchOffset_;
  std::string topicName_;
  std::string serviceName_;
  double fx_, fy_, fz_, tx_, ty_, tz_;

  BotaDriverTestResetService() : msgCount_(0)
  {
    meanWrenchOffset_.force.x = 0.0;
    meanWrenchOffset_.force.y = 0.0;
    meanWrenchOffset_.force.z = 0.0;
    meanWrenchOffset_.torque.x = 0.0;
    meanWrenchOffset_.torque.y = 0.0;
    meanWrenchOffset_.torque.z = 0.0;
  }

  ~BotaDriverTestResetService() override
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
  void resetWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
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

TEST_F(BotaDriverTestResetService, CustomWrench)
{
  SCOPED_TRACE("TestResetServiceCustomWrench");
  ros::Time time_offset;
  // add a small time offset for the serial to start
  double starting_offset = 18.0;
  double test_duration;
  ASSERT_EQ(nh_.getParam("topic_name", topicName_), true);
  ASSERT_EQ(nh_.getParam("service_name", serviceName_), true);
  ASSERT_EQ(nh_.getParam("fx", fx_), true);
  ASSERT_EQ(nh_.getParam("fy", fy_), true);
  ASSERT_EQ(nh_.getParam("fz", fz_), true);
  ASSERT_EQ(nh_.getParam("tx", tx_), true);
  ASSERT_EQ(nh_.getParam("ty", ty_), true);
  ASSERT_EQ(nh_.getParam("tz", tz_), true);
  ASSERT_EQ(nh_.getParam("test_duration", test_duration), true);
  ASSERT_GE(test_duration, starting_offset + 10.0);

  /*
   * Step 1
   * Create the previous offset of wrench.
   *
   */
  time_offset = ros::Time::now() + ros::Duration(starting_offset);
  while (ros::ok() && ros::Time::now() < time_offset)
  {
    ros::spinOnce();
    if (HasFatalFailure())
    {
      FAIL() << "Fatal errors occurred.\n";
      return;
    }
  }

  /*
   * Step 2
   * Call successfully the ros service (reset_wrench).
   *
   */
  ASSERT_TRUE(ros::service::waitForService(serviceName_));
  rokubimini_msgs::ResetWrench::Request req;
  rokubimini_msgs::ResetWrench::Response res;
  req.desired_wrench = geometry_msgs::Wrench();
  req.desired_wrench.force.x = fx_;
  req.desired_wrench.force.y = fy_;
  req.desired_wrench.force.z = fz_;
  req.desired_wrench.torque.x = tx_;
  req.desired_wrench.torque.y = ty_;
  req.desired_wrench.torque.z = tz_;
  ASSERT_TRUE(ros::service::call(serviceName_, req, res));
  EXPECT_EQ(res.success, true);

  /*
   * Step 3
   * Gather measurements to form the new mean offset of wrench.
   *
   */
  sub_ = nh_.subscribe(topicName_, 1000, &BotaDriverTestResetService::resetWrenchCallback,
                       (BotaDriverTestResetService*)this);
  time_offset = ros::Time::now() + ros::Duration(test_duration - starting_offset);
  while (ros::ok() && ros::Time::now() < time_offset)
  {
    ros::spinOnce();
    if (HasFatalFailure())
    {
      FAIL() << "Fatal errors occurred.\n";
      return;
    }
  }
  ASSERT_GT(msgCount_, 0U);
  EXPECT_NEAR(meanWrenchOffset_.force.x, fx_, 5.0);
  EXPECT_NEAR(meanWrenchOffset_.force.y, fy_, 5.0);
  EXPECT_NEAR(meanWrenchOffset_.force.z, fz_, 5.0);
  EXPECT_NEAR(meanWrenchOffset_.torque.x, tx_, 0.1);
  EXPECT_NEAR(meanWrenchOffset_.torque.y, ty_, 0.1);
  EXPECT_NEAR(meanWrenchOffset_.torque.z, tz_, 0.1);
}
}  // namespace bota_driver_testing