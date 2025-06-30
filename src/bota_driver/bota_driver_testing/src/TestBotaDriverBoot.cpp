/**
 * @authors     Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Tests Command
 */

#include <gtest/gtest.h>

#include <rosgraph_msgs/Log.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Wrench.h>
#include <ros/ros.h>
#include <ros/connection_manager.h>

namespace bota_driver_testing
{
class BotaDriverTestBoot : public ::testing::Test
{
protected:
  ros::Subscriber sub_;
  ros::NodeHandle nh_{ "~" };
  std::string nodeName_;
  float testDuration_;

  BotaDriverTestBoot()
  {
  }

  ~BotaDriverTestBoot() override
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
  void rosoutCallback(const rosgraph_msgs::Log::ConstPtr& msg)
  {
    // Check that the message has not type ERROR || FATAL and the sender node is not rokubimini.
    ASSERT_NE((msg->level == rosgraph_msgs::Log::ERROR || msg->level == rosgraph_msgs::Log::FATAL) &&
                  msg->name == nodeName_,
              true);
  }
};

TEST_F(BotaDriverTestBoot, NoErrorsInBoot)
{
  SCOPED_TRACE("NoErrorsInBoot");
  ASSERT_EQ(nh_.getParam("node", nodeName_), true);
  sub_ = nh_.subscribe("/rosout", 1000, &BotaDriverTestBoot::rosoutCallback, (BotaDriverTestBoot*)this);
  EXPECT_EQ(sub_.getNumPublishers(), 1U);
  ASSERT_EQ(nh_.getParam("test_duration", testDuration_), true);
  ros::Time time_after_boot = ros::Time::now() + ros::Duration(testDuration_);
  while (ros::ok() && ros::Time::now() < time_after_boot)
  {
    ros::spinOnce();
    if (HasFatalFailure())
    {
      FAIL() << "There were errors in the boot process.\nReceived message from rokubimini node with type ERROR or "
                "FATAL.";
      return;
    }
  }
}

// bool MESSAGE_RECEIVED_ONCE = false;
// void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
// {
//   // Every frame should have the following data:
//   // frame.data.forces[0] = 11.1;
//   // frame.data.forces[1] = 22.2;
//   // frame.data.forces[2] = 33.3;
//   // frame.data.forces[3] = 44.4;
//   // frame.data.forces[4] = 55.5;
//   // frame.data.forces[5] = 66.6;
//   MESSAGE_RECEIVED_ONCE = true;
//   ASSERT_FLOAT_EQ(msg->wrench.force.x, 11.1);
//   ASSERT_FLOAT_EQ(msg->wrench.force.y, 22.2);
//   ASSERT_FLOAT_EQ(msg->wrench.force.z, 33.3);
//   ASSERT_FLOAT_EQ(msg->wrench.torque.x, 44.4);
//   ASSERT_FLOAT_EQ(msg->wrench.torque.y, 55.5);
//   ASSERT_FLOAT_EQ(msg->wrench.torque.z, 66.6);
// }

// TEST_F(BotaDeviceDriverTest, CorrectSensorValuesInBoot)
// {
//   SCOPED_TRACE("CorrectSensorValuesInBoot");
//   sub_ = nh_.subscribe("/rokubimini/ft_sensor0/ft_sensor_readings/wrench", 1000,
//   &bota_device_driver::wrenchCallback);
//   //   30 seconds delay
//   ros::Time time_after_boot = ros::Time::now() + ros::Duration(30);
//   while (ros::ok() && ros::Time::now() < time_after_boot)
//   {
//     ros::spinOnce();
//     if (HasFatalFailure())
//     {
//       FAIL() << "There were errors in the frame data.\nReceived invalid data from rokubimini node publisher.";
//       return;
//     }
//   }
//   ASSERT_EQ(MESSAGE_RECEIVED_ONCE, true);
//   ros::shutdown();
// }

}  // namespace bota_driver_testing
