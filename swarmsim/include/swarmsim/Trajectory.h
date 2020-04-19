#ifndef trajectory_h
#define trajectory_h

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>

enum TrajContinuity {Continued = 0, Start, End};

struct Trajectory {
  std::vector<Eigen::Vector3d> pos; 
  std::vector<Eigen::Vector3d> vel;
  std::vector<Eigen::Vector3d> acc;
  std::vector<double> tList;
};


#endif