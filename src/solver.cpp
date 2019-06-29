#include "include/solver.h"
#include "ros/console.h"
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>

namespace mtg = mav_trajectory_generation;

Solver::Solver(int nDrones, double maxVel, double maxAcc, int nChecks,
               double frequency)
    : K(nDrones), maxVel(maxVel), maxAcc(maxAcc), nChecks(nChecks) {
  dt = 1 / frequency;
}

vector<Trajectory> Solver::solve(vector<Trajectory> droneWpts, bool start, bool end) {
  vector<Trajectory> trajList;
  for (int k = 0; k < K; k++) {
    Trajectory t_k = droneWpts[k];
    vector<double> tList = droneWpts[k].tList;
    
    mav_trajectory_generation::Vertex::Vector vertices;
    const int derivative_to_optimize =
        mav_trajectory_generation::derivative_order::SNAP;
    for(int i=0;i<t_k.pos.size();i++) {
        Eigen::Vector3d pos = t_k.pos[i];
        mav_trajectory_generation::Vertex v(3);
        if(start && i == 0) {
          v.makeStartOrEnd(pos, derivative_to_optimize);
        }
        else if(end && i == t_k.pos.size()-1) {
          v.makeStartOrEnd(pos, derivative_to_optimize);
        }
        else {
          v.addConstraint(mtg::derivative_order::POSITION, pos);
        }
        vertices.push_back(v);
    }
    mtg::NonlinearOptimizationParameters parameters;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<10> opt(3, parameters);
    opt.setupFromVertices(vertices, tList, derivative_to_optimize);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, maxVel);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, maxAcc);
    opt.optimize();
    mav_trajectory_generation::Trajectory trajectory;
    opt.getTrajectory(&trajectory);
  }

  return trajList;
}

// Trajectory Solver::calculateTrajectory(vector<double> coef, double t0,
//                                        double t1) {
//   Trajectory traj;
//   MatrixXf xc(n, 1);
//   MatrixXf yc(n, 1);
//   MatrixXf zc(n, 1);

//   xc << coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6];
//   yc << coef[7], coef[8], coef[9], coef[10], coef[11], coef[12], coef[13];
//   zc << coef[14], coef[15], coef[16], coef[17], coef[18], coef[19], coef[20];

//   for (int i = 1; i <= coef.size(); i++) {
//     cout << coef[i - 1] << " ";
//     if (i % 7 == 0)
//       cout << endl;
//   }

//   for (double t = t0; t <= t1; t += dt) {
//     Vector3d pos;
//     MatrixXf posT = getPosTimeVec(t);
//     pos[0] = posT.row(0) * xc.col(0);
//     pos[1] = posT.row(0) * yc.col(0);
//     pos[2] = posT.row(0) * zc.col(0);
//     traj.pos.push_back(pos);
//   }
//   ROS_DEBUG_STREAM("Calculated XYZ trajectories");
//   return traj;
// }
