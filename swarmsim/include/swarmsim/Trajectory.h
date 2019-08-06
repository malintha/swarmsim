#ifndef trajectory_h
#define trajectory_h

#include <iostream>
#include <eigen3/Eigen/Dense>

enum TrajContinuity {Continued = 0, Start, End};

struct Trajectory {
  std::vector<Eigen::Vector3d> pos; 
  std::vector<double> tList;
};


#endif