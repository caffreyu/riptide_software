#include "riptide_hardware/test_sub_pub.h"

#define gou 1

int main(int argc, char ** argv) {
    ros::init(argc, argv, "test_sub_pub");
    TestSubPub TSB;
    TSB.Loop();
}

TestSubPub::TestSubPub():nh("test_sub_pub") {
    sub = nh.subscribe<riptide_msgs::test>("random/topic", 1, &TestSubPub::TestCB, this);
    pub = nh.advertise<riptide_msgs::test>("random/topic", 1);
    state.value = 0;

}

void TestSubPub::TestCB(const riptide_msgs::test::ConstPtr& msg) {
    ROS_INFO("Ruru shi gou");                                                                                    
}

void TestSubPub::Loop() {
    ros::Rate rate(10);
    while (! ros::isShuttingDown()) {
        ros::spinOnce();
        rate.sleep();
    }
}