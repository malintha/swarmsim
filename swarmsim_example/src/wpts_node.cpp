#include <ros/ros.h>
#include "eigen3/Eigen/Dense"
#include "dji_sdk/dji_sdk.h"
#include "geometry_msgs/PoseArray.h"
#include "WptsNavigate.h"
#include "swarmsim/state.h"

using namespace std;

void WptsNavigate::run() {
    if(drone->getState() == States::Armed) {
        drone->TOLService(true);
    }
    else if(drone->getState() == States::Autonomous) {
        drone->move();
    }
}

WptsNavigate::WptsNavigate(ros::NodeHandle &nh):nh(nh) {
    drone = new Drone(0, nh);
    drone->setState(States::Armed);
    nh.subscribe("local_way_points", 10, &WptsNavigate::localGoalCB, this);
    takenOff = false;

}

void WptsNavigate::localGoalCB(const geometry_msgs::PoseArrayConstPtr &msg) {
    geometry_msgs::PoseArray pArr = *msg;
    if(drone->wptsList.size() == 0) {
        vector<geometry_msgs::Pose> selectedP = preProcessWpts(pArr);
        drone->addWaypoints(selectedP);
    }
    else {
        ROS_DEBUG_STREAM("Recieved, but not adding as queue is not empty");
    }
}

vector<geometry_msgs::Pose> WptsNavigate::preProcessWpts(geometry_msgs::PoseArray p_arr) {
    vector<geometry_msgs::Pose> wpts;
    int nPts = 0;
    p_arr.poses.size() > 5 ? nPts = 5 : nPts = p_arr.poses.size();
    for(int i=0 ; i < nPts; i++) {
        wpts.push_back(p_arr.poses[i]);
    }
    return wpts;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "wpts_node");
    ros::NodeHandle nh;
    WptsNavigate navigate(nh);
    while(ros::ok()) {
        navigate.run();
    }
}
