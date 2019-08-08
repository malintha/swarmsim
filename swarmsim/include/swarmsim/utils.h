#include <qpOASES.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <algorithm>
#include "Trajectory.h"
#include "ros/ros.h"
#include <future>
#include <stdexcept>
#include "YamlDescriptor.h"

using namespace std;

namespace simutils {

    void processYamlFile(char *fPath, YamlDescriptor &yamlDescriptor);

    vector<double> loadTimesFromFile(ros::NodeHandle &nh);

    vector<Trajectory> loadTrajectoriesFromFile(int n_drones, ros::NodeHandle &nh, string trajDir);

    int getGazeboModelId(string* modelNames, string elementName);

    void parseYamlHeader(char *fPath, YamlDescriptor &yamlDescriptor);

    vector<Trajectory> getHorizonTrajetories(int horizonId, YamlDescriptor yamlDescriptor);

}