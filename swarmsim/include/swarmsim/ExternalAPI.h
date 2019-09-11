#ifndef EXTERNAL_API_H
#define EXTERNAL_API_H

#include <iostream>
#include "nav_msgs/Odometry.h"
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <ros/console.h>
#include "state.h"

using namespace std;
using namespace Eigen;

enum APIType {
    MAVROS = 0, DJIType
};

class ExternalAPI {
    public:
    ExternalAPI();
    ExternalAPI(int apiType, int droneId);
    virtual bool armDrone(bool arm);
    virtual bool TOL(bool takeoff);
    virtual bool sendSetPoint(geometry_msgs::PoseStamped pose);

    string getLocalPositionTopic();
    string getGlobalPositionTopic();
    string getSetPointTopic();

    Vector3d getLocalPosition();
    Vector3d getGlobalPosition();
    virtual Vector3d getLocalWaypoint(Vector3d waypoint);

    // bool getIsReady();
    void ready(bool ready);
    void setState(int state);
    int getState(); 
    void setTakeoffHeight(double takeoffHeight);

    int state;
    int droneId;
    int apiType;
    ros::Subscriber globalPositionSub;
    ros::Subscriber localPositionSub;

    Vector3d localPos;
    Vector3d globalPos;
    Vector3d initGlobalPos;
    float yaw;
    double takeoffHeight;


};

#endif