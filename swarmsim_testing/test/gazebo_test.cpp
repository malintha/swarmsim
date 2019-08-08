#include <gtest/gtest.h>
#include "ros/ros.h"
#include "testutils.h"
#include "ros/console.h"

TEST_F(TestUtils, testTwoRobots) {

    mavrosStateSub = nh.subscribe("/1/mavros/state",1, &TestUtils::mavrosStateCB, (TestUtils*)this);
    swarmStateSub = nh.subscribe("/swarmsim_example/swarm/state",10, &TestUtils::swarmStateCB, (TestUtils*)this);
    modelStatesSub = nh.subscribe("/gazebo/model_states",10, &TestUtils::gazeboModelStateCB, (TestUtils*)this);
    while(!guided) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    getAssertionValuesFromFile(filePath);
    int nHorizons = goalset.size();
    ros::Time time = ros::Time::now();
    for(int i=0;i<nHorizons;i++) {
        vector<double> checkpoints = yamlDescriptor.getTimesArray()[i].times;
        vector<double> cumulativeTimes =  getCumulativeTime(checkpoints);
        double horzLength = cumulativeTimes[cumulativeTimes.size() - 1];
        prevHorzLength = horzLength;

        double curr_sec = 0;
        int pos_id = 0;

        while(curr_sec <= horzLength) {
            ros::Duration(0.5).sleep();
            if(swarmState == States::Autonomous) {
                if(doubleEqual(curr_sec, cumulativeTimes[pos_id])) {
                    for(int k=0;k<2;k++) {
                        bool inPlace = assertPosition(i, pos_id, k);
                        ROS_DEBUG_STREAM("Drone: "<<k <<" in place: "<<inPlace);
                        ASSERT_TRUE(inPlace);
                    }
                    pos_id++;
                }
                curr_sec += 0.5;
            }
            else if(swarmState == States::Reached) {
                break;
            }
            ros::spinOnce();
        }
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "gazebosimtest");

    int result = RUN_ALL_TESTS();
    return result;
}