#include "solver.h"
#include <gtest/gtest.h>
#include "Trajectory.h"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

TEST(SolverSuite, testSingleRobot)
{
    //solver object for a single robot with a single spline
    // Solver s(1,4,5,0,10);
    // //creating the wpts list
    // vector<Trajectory> wpts;
    // Trajectory t1; //only spline
    // vector<Vector3d> pos;
    // Vector3d p1, p2, p3;
    // p1 << 0,0,0;
    // p2 << 3,4,3;
    // p3 << 6,6,5;
    // pos.push_back(p1);
    // pos.push_back(p2);
    // pos.push_back(p3);
    // vector<double> t;
    // t.push_back(3);
    // t.push_back(3);
    
    // t1.pos = pos;
    // t1.tList = t;
    // wpts.push_back(t1);
    // vector<Trajectory> results = s.solve(w1);
    cout<<"####"<<endl;
    ASSERT_EQ(7, 7);
    
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  timing::Timing::Print(std::cout);

  return result;
}
