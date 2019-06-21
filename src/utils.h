#include <qpOASES.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <algorithm> 
#include "Trajectory.h"
#include "ros/ros.h"
#include <future>

using namespace std;
USING_NAMESPACE_QPOASES

namespace simutils {
    void blockDiag(vector<int> *H, real_t *Hn, int HnRows);
    void printmat(vector<int> *H);
    void reSizeMat(vector<int> *A, int prevDim, int newDim);

    // future<vector<Trajectory> > getTrajectoryList(char* fPath, int horizon_id);
    vector<Trajectory> processYamlFile(char* fPath, int horizon_id);


    vector<double> loadTimesFromFile(ros::NodeHandle &nh);
    vector<Trajectory> loadTrajectoriesFromFile(int n_drones, ros::NodeHandle &nh, bool fullTrajecory);
}