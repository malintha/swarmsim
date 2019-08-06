#include <qpOASES.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <algorithm>
#include "Trajectory.h"
#include "ros/ros.h"
#include <future>
#include <stdexcept>

using namespace std;

namespace simutils {

    void processYamlFile(char *fPath, int horizon_id, int &horizons, vector<Trajectory> &goalPoints);

    vector<double> loadTimesFromFile(ros::NodeHandle &nh);

    vector<Trajectory> loadTrajectoriesFromFile(int n_drones, ros::NodeHandle &nh, string trajDir);

    int getGazeboModelId(string* modelNames, string elementName);
}