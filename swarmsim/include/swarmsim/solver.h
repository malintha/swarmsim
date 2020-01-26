#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <algorithm>
// #include <qpOASES.hpp>
#include "Trajectory.h"
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

using namespace std;
using namespace Eigen;

class Solver {
    public:
        Solver(int nDrones, double maxVel, double maxAcc, double frequency);
        vector<Trajectory> solve(vector<Trajectory> droneWpts, bool initial, bool last, std::vector<Trajectory> prevPlan);
        
    private:
        int n = 7;
        int K = 1;
        int D = 3;
        double maxVel;
        double maxAcc;
        int nChecks;
        double dt = 0.1;
        int getnVariables();
        int getnConstraints();
        MatrixXf getHblock(double t0, double t1);
        MatrixXf getPosTimeVec(double t);
        MatrixXf getVelTimeVec(double t);
        MatrixXf getAccTimeVec(double t);
        Trajectory calculateTrajectoryWpts(mav_trajectory_generation::Trajectory& traj);

        int nwpts = 0;

};