#ifndef trajectory_h
#define trajectory_h

#include <iostream>
#include <eigen3/Eigen/Dense>

struct Trajectory {
  std::vector<Eigen::Vector3d> pos; 
};


#endif