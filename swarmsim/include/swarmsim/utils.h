#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <algorithm>
#include "Trajectory.h"
#include "ros/ros.h"
#include <future>
#include <stdexcept>
#include "YamlDescriptor.h"
#include "geometry_msgs/Quaternion.h"

using namespace std;

namespace simutils {

    void processYamlFile(char *fPath, YamlDescriptor &yamlDescriptor);

    vector<double> loadTimesFromFile(ros::NodeHandle &nh);

    vector<Trajectory> loadTrajectoriesFromFile(int n_drones, ros::NodeHandle &nh, string trajDir);

    int getGazeboModelId(string* modelNames, string elementName);

    void parseYamlHeader(char *fPath, YamlDescriptor &yamlDescriptor);

    vector<Trajectory> getHorizonTrajetories(int horizonId, YamlDescriptor yamlDescriptor);

    Eigen::Vector3d getRPY(geometry_msgs::Quaternion orientation);

    float getEucDistance(Eigen::Vector3d p1, Eigen::Vector3d p2);

}