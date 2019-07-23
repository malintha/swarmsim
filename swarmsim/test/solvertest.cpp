#include "solver.h"
#include <gtest/gtest.h>
#include "Trajectory.h"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

Trajectory getTestingTrajectory(Vector3d offset) {
    Trajectory tr; //only spline
    vector<Vector3d> pos;
    Vector3d p1, p2, p3;
    p1 << 0,0,0;
    p2 << 3,3,3;
    p3 << 6,6,6;

    p1 = p1+offset;
    p2 = p2+offset;
    p3 = p3+offset;

    pos.push_back(p1);
    pos.push_back(p2);
    pos.push_back(p3);
    vector<double> t;
    t.push_back(3);
    t.push_back(3);
    tr.pos = pos;
    tr.tList = t;
    return tr;
}

TEST(SwarmSimTestSuite, testSingleRobot) {
    //solver object for a single robot
    Solver s(1,4,5,10);
    vector<Trajectory> wpts;
    Vector3d offset;
    offset << 0,0,0;
    wpts.push_back(getTestingTrajectory(offset));
    vector<Trajectory> results = s.solve(wpts);
    ASSERT_GT(results[0].pos.size(), 0); 
}

TEST(SwarmSimTestSuite, testMultiRobot) {
    //solver object for two robots
    Solver s(2,4,5,10);
    vector<Trajectory> wpts;
    Vector3d offset;
    offset << 0,0,0;
    wpts.push_back(getTestingTrajectory(offset));
    offset << 1,0,0;
    wpts.push_back(getTestingTrajectory(offset));
    vector<Trajectory> results = s.solve(wpts);
    ASSERT_EQ(results.size(), 2); 
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
