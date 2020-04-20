#ifndef Obstacle_h
#define Obstacle_h

#include <iostream>
#include <eigen3/Eigen/Dense>

struct Obstacle
{
    Eigen::Vector3d center;
    float height;
    float width;
    float length;

    /**
     * check if a given point is within the obstacle
    */
    bool isWithin(Eigen::Vector3d pt) {}
};


#endif