#ifndef TEST_PUB_SUB_H
#define TEST_PUB_SUB_H
//#define DEPTH_OFFSET 0.1 // Save, these were used by arduino
//#define DEPTH_SLOPE 1

#include "ros/ros.h"
#include "riptide_msgs/test.h"
using namespace std;

class TestPubSub
{

private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::Subscriber sub2;
  ros::Publisher pub2;

  // IIR LPF Variables
  int prev_value, x, y, z;

  riptide_msgs::test value_state;
public:
  TestPubSub();
  template <typename T>
  void LoadParam(string param, T &var);
  void testCB(const riptide_msgs::test::ConstPtr& msg);
  void SmoothDataIIR();
  void Loop();
};

#endif
