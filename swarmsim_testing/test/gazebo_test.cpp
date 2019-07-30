#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "swarmsim/utils.h"
#include "swarmsim/Trajectory.h"

using namespace std;

class testUtils {

    public:
        testUtils(int nDrones) {
            ros::NodeHandle n;
            modelStatesSub = n.subscribe("/gazebo/model_states", 10, &testUtils::gazeboModelStateCB, this);
            n.getParam("/swarmsim_testing/trajDir_testing", trajDir);
            n.getParam("/swarmsim_testing/yamlFileName_testing", yamlFileName);
        }

        void gazeboModelStateCB(const gazebo_msgs::ModelStatesConstPtr& msg) {
            pose_vec = msg->pose;
        }

        geometry_msgs::Pose getModelPose(int robotId) {
            return pose_vec[robotId];
        }

        bool assertPosition(int posId) {
            // for(int i=0;i<nDrones;i++) {
                int i = 0; //hardcoding the drone num
                vector<Eigen::Vector3d> targetPosList = goalset[i].pos;
                Eigen::Vector3d targetPos = targetPosList[posId];
                geometry_msgs::Pose robot_pose = pose_vec[2+i]; //first 2 elements of the list are asphalt plane and height map
                Eigen::Vector3d currPos;
                currPos << robot_pose.point.x, robot_pose.point.y, robot_pose.point.z;
                return withinRadius(currPos, targetPos);
                //todo: needs to change the times to be cumulative
            // }
        }

        bool withinRadius(Eigen::Vector3d p1, Eigen::Vector3d p2) {
            return (p1 - p2).norm() < 1;
        }

        vector<Trajectory> getAssertionValuesFromFile(int horizon) {
            char cstr[yamlFileName.size() + 1];
            copy(yamlFileName.begin(), yamlFileName.end(), cstr);
            cstr[yamlFileName.size()] = '\0';
            int horizons;
            vector<Trajectory> goals; 
            this->goalset = goals;
            simutils::processYamlFile(cstr, horizon, horizons, goals);
            return goals;
        }

    private:
        vector<Trajectory> goalset;
        string trajDir;
        string yamlFileName;
        ros::NodeHandle n;
        int nDrones;
        ros::Subscriber modelStatesSub;
        vector<geometry_msgs::Pose> pose_vec;



};



TEST(GazeboSimTestSuite, testTwoRobots) {
    testUtils tu(2);
    vector<Trajectory> tr_0 = tu.getAssertionValuesFromFile(0);
    vector<double> times_list = tr_0.tList;
    for(int i=0; i<times_list; i++) {
        double target_time = times_list[i];
        ros::Time time = ros::Time::now();
        double curr_sec = time.toSec();
        do {
            time = ros::Time::now();
            curr_sec = time.toSec();
        }
        while(curr_sec < target_time);
        tu.assertPosition();

    }


    if(time.toSec() <= 0.5) {
        geometry_msgs::Pose p1 = tu.getModelPose(0);
        geometry_msgs::Pose p2 = tu.getModelPose(1);

    }
    ASSERT_GT(1, 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  return result;
}