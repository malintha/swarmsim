#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "swarmsim/utils.h"
#include "swarmsim/Trajectory.h"
#include "swarmsim/state.h"
#include "mavros_msgs/State.h"
#include "ros/console.h"
#include <gtest/gtest.h>
#include <std_msgs/Int8.h>

using namespace std;

class TestUtils : public testing::Test {

    public:
        ros::NodeHandle nh;
        ros::Subscriber mavrosStateSub;
        vector<vector<Trajectory> > goalset;
        string trajDir;
        string yamlFileName;
        string filePath;
        int nDrones;
        ros::Subscriber modelStatesSub;
        vector<geometry_msgs::Pose> pose_vec;
        bool guided;
        double prevHorzLength;
        int swarmState;
        ros::Subscriber swarmStateSub;
        vector<Eigen::Vector3d> initPositions;
        vector<vector<bool> > wptPasses;
        vector<int> gazeboPoseIdList;
        YamlDescriptor yamlDescriptor;

        TestUtils() {
            guided = false;
            nh.getParam("/swarmsim_example/trajDir", trajDir);
            nh.getParam("/swarmsim_example/yamlFileName", yamlFileName);
            nDrones = 2;
            stringstream ss;
            ss << trajDir<<yamlFileName;
            filePath = ss.str();
            prevHorzLength = 0;
        }

        void gazeboModelStateCB(const gazebo_msgs::ModelStatesConstPtr& msg) {
            pose_vec = msg->pose;
            if(gazeboPoseIdList.size() == 0) {
                ros::V_string gazeboPoseNameList = msg->name;
                for(int i=0; i < nDrones; i++) {
                    string droneName = getGazeboDroneName(i);
                    for(int j=0;j<gazeboPoseNameList.size();j++) {
                        string key = gazeboPoseNameList[j];
                        if(droneName.compare(key) == 0) {
                            gazeboPoseIdList.push_back(j);
                        }
                    }
                }
            }
        }

        void mavrosStateCB(const mavros_msgs::StateConstPtr& msg) {
            guided = msg->guided;
            if(guided) {
                this->mavrosStateSub.shutdown();
            }
        }

        void swarmStateCB(const std_msgs::Int8ConstPtr& msg) {
            swarmState = msg->data;
        }

        geometry_msgs::Pose getModelPose(int robotId) {
            return pose_vec[robotId];
        }

        void getInitPositions() {
            for(int i=0;i<nDrones;i++) {
                geometry_msgs::Pose robot_pose = pose_vec[2+i];
                Eigen::Vector3d currPos;
                currPos << robot_pose.position.x, robot_pose.position.y, robot_pose.position.z;
                initPositions.push_back(currPos);
            }
        }

        bool assertPosition(int horzid, int posId, int droneId) {
            DroneTrajectory drone_tr = yamlDescriptor.getdroneTrajectories()[droneId];

            vector<Eigen::Vector3d> targetPosList = drone_tr.horzTrajList[horzid].pos;
            Eigen::Vector3d targetPos = targetPosList[posId];
            geometry_msgs::Pose robot_pose = pose_vec[gazeboPoseIdList[droneId]];
            Eigen::Vector3d currPos;
            currPos << robot_pose.position.x, robot_pose.position.y, robot_pose.position.z;
            return withinRadius(currPos, targetPos);
        }

        string getGazeboDroneName(int id) {
            stringstream ss;
            ss << "iris_" << id;
            return ss.str();
        }

        bool withinRadius(Eigen::Vector3d p1, Eigen::Vector3d p2) {
            return (p1 - p2).norm() < yamlDescriptor.getMovingThreshold();
        }

        vector<double> getCumulativeTime(vector<double> tList) {
            double cumulative = 0;
            vector<double> cumulativeList;
            cumulativeList.push_back(cumulative);
            for(int i=0;i<tList.size();i++) {
                cumulative += tList[i];
                cumulativeList.push_back(cumulative);
            }
            return cumulativeList;
        }

        bool doubleEqual(double a, double b) {
            return abs(a-b) < 0.001;
        }

        void getAssertionValuesFromFile(string yamlFilePath) {
            ROS_DEBUG_STREAM("file path: "<<yamlFilePath);
            char cstr[yamlFilePath.size() + 1];
            copy(yamlFilePath.begin(), yamlFilePath.end(), cstr);
            cstr[yamlFilePath.size()] = '\0';
            simutils::processYamlFile(cstr, yamlDescriptor);
            ROS_DEBUG_STREAM("Loaded the YamlDescriptor");
        }
};