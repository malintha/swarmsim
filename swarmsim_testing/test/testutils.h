#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "swarmsim/utils.h"
#include "swarmsim/Trajectory.h"

using namespace std;

class TestUtils {

    public:
        TestUtils(int nDrones) {
            ros::NodeHandle n("~");
            modelStatesSub = n.subscribe("/gazebo/model_states", 10, &TestUtils::gazeboModelStateCB, this);
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
                currPos << robot_pose.position.x, robot_pose.position.y, robot_pose.position.z;
                return withinRadius(currPos, targetPos);
            // }
        }

        bool withinRadius(Eigen::Vector3d p1, Eigen::Vector3d p2) {
            return (p1 - p2).norm() < 1;
        }

        vector<double> getCumulativeTime(vector<double> tList) {
            double cumulative = 0;
            vector<double> cumulativeList;
            for(int i=0;i<tList.size();i++) {
                cumulative += tList[i];
                cumulativeList.push_back(cumulative);
            }
            return cumulativeList;
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