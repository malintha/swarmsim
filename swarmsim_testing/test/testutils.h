#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "swarmsim/utils.h"
#include "swarmsim/Trajectory.h"
#include "mavros_msgs/State.h"
#include "ros/console.h"
#include <gtest/gtest.h>

using namespace std;

class TestUtils : public testing::Test {

    public:
        ros::NodeHandle nh;
        ros::Subscriber mavrosStateSub;
        vector<vector<Trajectory> > goalset;
        string trajDir;
        string yamlFileName;
        int nDrones;
        ros::Subscriber modelStatesSub;
        vector<geometry_msgs::Pose> pose_vec;
        bool guided;

        TestUtils() {
            guided = false;
        }

        void gazeboModelStateCB(const gazebo_msgs::ModelStatesConstPtr& msg) {
            pose_vec = msg->pose;
        }

        void mavrosStateCB(const mavros_msgs::StateConstPtr& msg) {
            guided = msg->guided;
            ROS_ERROR_STREAM("Mavros Guided CB");
            if(guided) {
                ROS_ERROR_STREAM("shutting down CB");
                this->mavrosStateSub.shutdown();
            }
        }

        // geometry_msgs::Pose getModelPose(int robotId) {
        //     return pose_vec[robotId];
        // }

        // bool assertPosition(int posId) {
        //     // for(int i=0;i<nDrones;i++) {
        //         int i = 0; //hardcoding the drone num
        //         vector<Eigen::Vector3d> targetPosList = goalset[i].pos;
        //         Eigen::Vector3d targetPos = targetPosList[posId];
        //         geometry_msgs::Pose robot_pose = pose_vec[2+i]; //first 2 elements of the list are asphalt plane and height map
        //         Eigen::Vector3d currPos;
        //         currPos << robot_pose.position.x, robot_pose.position.y, robot_pose.position.z;
        //         return withinRadius(currPos, targetPos);
        //     // }
        // }

        // bool withinRadius(Eigen::Vector3d p1, Eigen::Vector3d p2) {
        //     return (p1 - p2).norm() < 1;
        // }

        // vector<double> getCumulativeTime(vector<double> tList) {
        //     double cumulative = 0;
        //     vector<double> cumulativeList;
        //     for(int i=0;i<tList.size();i++) {
        //         cumulative += tList[i];
        //         cumulativeList.push_back(cumulative);
        //     }
        //     return cumulativeList;
        // }

        vector<Trajectory> getAssertionValuesFromFile(string yamlFilePath) {
            // ROS_ERROR_STREAM("file path: "<<yamlFilePath);

            char cstr[yamlFileName.size() + 1];
            copy(yamlFileName.begin(), yamlFileName.end(), cstr);
            cstr[yamlFileName.size()] = '\0';
            int horizons;
            vector<Trajectory> goals;
            simutils::processYamlFile(cstr, 0, horizons, goals);
            ROS_ERROR_STREAM("N_HORIZONS: "<<goalset.size());

            for(int i=0;i<horizons;i++) {
                simutils::processYamlFile(cstr, i, horizons, goals);
                this->goalset.push_back(goals);
            }
            ROS_ERROR_STREAM("N_HORIZONS: "<<goalset.size());
            
            return goals;
        }

        // bool readyDrones() {
        //     return ready;
        // }

        // protected:

};