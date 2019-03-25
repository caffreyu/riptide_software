#ifndef TEST_SUB_PUB_H
#define TEST_SUB_PUB_H

#include "ros/ros.h"

#include "riptide_msgs/test.h"
using namespace std;

class TestSubPub 
{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;  // Sub should has four paramters
  ros::Publisher pub;   // Pub should has two parameters, the first two of the subscriber

  riptide_msgs::test state;

public:
  TestSubPub();         // Class constructor
  void TestCB(const riptide_msgs::test::ConstPtr& msg);
  void Loop();
};

#endif