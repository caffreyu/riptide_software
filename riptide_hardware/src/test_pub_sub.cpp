#include "riptide_hardware/test_pub_sub.h"


int main(int argc, char** argv)
{
 ros::init(argc, argv, "test_pub_sub");
 TestPubSub dp;
 dp.Loop();
}

// Constructors
TestPubSub::TestPubSub() : nh("test_pub_sub") 
{
 sub = nh.subscribe<riptide_msgs::test>("test/topic1", 1, &TestPubSub::testCB, this);
 pub = nh.advertise<riptide_msgs::test>("test/topic1", 1);
 sub2 = nh.subscribe<riptide_msgs::test>("test/topic2", 1, &TestPubSub::testCB, this);
 pub2 = nh.advertise<riptide_msgs::test>("test/topic2", 1);
 TestPubSub::LoadParam<int>("x", x);
 TestPubSub::LoadParam<int>("y", y);
 TestPubSub::LoadParam<int>("z", z);
 value_state.value = 100;
}

// Load parameter from namespace
template <typename T>
void TestPubSub::LoadParam(string param, T &var)
{
  try
  {
    if (!nh.getParam(param, var))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    string ns = nh.getNamespace();
    // ROS_ERROR("Depth Processor Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

// Callback
void TestPubSub::testCB(const riptide_msgs::test::ConstPtr& test_msg)
{
  value_state.value = test_msg -> value;
  TestPubSub::SmoothDataIIR();
  ROS_INFO("The value is [%d]", value_state.value);

  // Publish smoothed data to state/test
  pub2.publish(value_state);
}

void TestPubSub::SmoothDataIIR() {
  if (value_state.value < 200) {
    value_state.value = prev_value + x * y + z;
    prev_value = value_state.value;
  }
  else {
    value_state.value = 0;
    prev_value = value_state.value;
  }
}

void TestPubSub::Loop()
{
  ros::Rate rate(1);
  while (!ros::isShuttingDown()){
    pub.publish(value_state);
    //pub2.publish(value_state);
    ros::spinOnce();
    rate.sleep();
  }
}
