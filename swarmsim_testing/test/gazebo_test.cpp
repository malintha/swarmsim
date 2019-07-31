#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "geometry_msgs/Pose.h"
#include "swarmsim/utils.h"
#include "swarmsim/Trajectory.h"
#include "testutils.h"

using namespace std;

TEST(GazeboSimTestSuite, testTwoRobots) {
    // TestUtils tu(2);
    boost::shared_ptr<gazebo_msgs::ModelStates const> feedback;
    feedback = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states",ros::Duration(100.0));
    
    // vector<Trajectory> tr_0 = tu.getAssertionValuesFromFile(0);
    // vector<double> times_list = tu.getCumulativeTime(tr_0[0].tList);

    // for(int i=0; i<times_list.size(); i++) {
    //     double target_time = times_list[i];
    //     ros::Time time = ros::Time::now();
    //     double curr_sec = time.toSec();
    //     do {
    //         time = ros::Time::now();
    //         curr_sec = time.toSec();
    //     }
    //     while(curr_sec < target_time);
    //     cout<<"####assert position"<<endl;
    //     // tu.assertPosition();

    // }


    // if(time.toSec() <= 0.5) {
    //     geometry_msgs::Pose p1 = tu.getModelPose(0);
    //     geometry_msgs::Pose p2 = tu.getModelPose(1);

    // }
    // ASSERT_EQ(1, 1);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "gazebosimtest");

    int result = RUN_ALL_TESTS();
    return result;
}