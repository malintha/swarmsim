#include "solver.h"
#include "ros/console.h"
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

namespace mtg = mav_trajectory_generation;

Solver::Solver(int nDrones, double maxVel, double maxAcc, int nChecks,
               double frequency)
        : K(nDrones), maxVel(maxVel), maxAcc(maxAcc), nChecks(nChecks) {
    dt = (double) 1 / frequency;
}

vector<Trajectory> Solver::solve(vector<Trajectory> droneWpts, bool start, bool end) {
    vector<Trajectory> trajList;
    for (int k = 0; k < K; k++) {
        Trajectory t_k = droneWpts[k];
        vector<double> tList = droneWpts[k].tList;

        mav_trajectory_generation::Vertex::Vector vertices;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
        for (int i = 0; i < t_k.pos.size(); i++) {
            Eigen::Vector3d pos = t_k.pos[i];
            mav_trajectory_generation::Vertex v(3);
            if (i == 0 || i == t_k.pos.size() - 1) {
                Eigen::Vector3d vel;
                vel << 0,0,0;
                v.makeStartOrEnd(pos, derivative_to_optimize);
                v.addConstraint(mtg::derivative_order::VELOCITY, vel);
            } else {
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
        trajList.push_back(calculateTrajectoryWpts(trajectory));
    }
    return trajList;
}

Trajectory Solver::calculateTrajectoryWpts(mtg::Trajectory& traj) {
    mav_msgs::EigenTrajectoryPoint::Vector flat_states;
    mav_trajectory_generation::sampleWholeTrajectory(traj, dt, &flat_states);
    Trajectory tr;
    for (auto & flat_state : flat_states) {
        tr.pos.push_back(flat_state.position_W);
    }
    return tr;
}