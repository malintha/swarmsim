#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <algorithm>
#include <qpOASES.hpp>
#include "Trajectory.h"
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

using namespace std;
using namespace Eigen;
USING_NAMESPACE_QPOASES

class Solver {
    public:
        Solver(int nDrones, double maxVel, double maxAcc, int nChecks, double frequency);
        vector<Trajectory> solve(vector<Trajectory> droneWpts, bool initial, bool end);
        
    private:
        int n = 7;
        int K = 1;
        int M = 1;
        int D = 3;
        double maxVel;
        double maxAcc;
        int nChecks;
        double dt = 0.1;
        SQProblem* sqp;
        QProblem* qp;
        int getnVariables();
        int getnConstraints();
        MatrixXf getHblock(double t0, double t1);
        MatrixXf getPosTimeVec(double t);
        MatrixXf getVelTimeVec(double t);
        MatrixXf getAccTimeVec(double t);
        real_t* matrix2realt(MatrixXf mat);
        Trajectory calculateTrajectory(vector<double> coefficients,double t0, double t1);

        int nwpts = 0;

};