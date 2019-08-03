#include <gtest/gtest.h>
#include "ros/ros.h"
#include "testutils.h"
#include "ros/console.h"

TEST_F(TestUtils, testTwoRobots) {

    mavrosStateSub = nh.subscribe("/1/mavros/state",1, &TestUtils::mavrosStateCB, (TestUtils*)this);
    while(!guided) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    nh.getParam("/swarmsim_testing/trajDir_testing", trajDir);
    nh.getParam("/swarmsim_testing/yamlFileName_testing", yamlFileName);
    ROS_ERROR_STREAM("###file path: "<<yamlFileName);


    ROS_DEBUG_STREAM("Drones are guided");


}

// TEST_F(TestUtils, testTwoRobots) {

//     //subscribe to the last drone state

//     ros::Duration(10).sleep();
//     // boost::shared_ptr<gazebo_msgs::ModelStates const> feedback;
//     // feedback = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/1/mavros/state",ros::Duration(100.0));
//     // vector<Trajectory> tr_0 = tu.getAssertionValuesFromFile(0);
//     // vector<double> times_list = tu.getCumulativeTime(tr_0[0].tList);
//     // do {
//     //     cout<<"waiting "<<endl;
//     // }
//     // while(!tu.readyDrones());
//     // ROS_ERROR_STREAM("#####finishing+ "<<tu.readyDrones());

//     // for(int i=0; i<times_list.size(); i++) {
//     //     double target_time = times_list[i];
//     //     ros::Time time = ros::Time::now();
//     //     double curr_sec = time.toSec();
//     //     do {
//     //         time = ros::Time::now();
//     //         curr_sec = time.toSec();
//     //     }
//     //     while(curr_sec < target_time);
//     //     cout<<"####assert position"<<endl;
//     //     // tu.assertPosition();

//     // }


//     // if(time.toSec() <= 0.5) {
//     //     geometry_msgs::Pose p1 = tu.getModelPose(0);
//     //     geometry_msgs::Pose p2 = tu.getModelPose(1);

//     // }
//     // ASSERT_EQ(1, 0);
// }

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "gazebosimtest");

    int result = RUN_ALL_TESTS();
    return result;
}