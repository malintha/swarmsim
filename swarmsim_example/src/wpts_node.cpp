#include <ros/ros.h>
#include "eigen3/Eigen/Dense"
#include "dji_sdk/dji_sdk.h"
#include "geometry_msgs/PoseArray.h"
#include "WptsNavigate.h"
#include "swarmsim/state.h"
#include "ros/console.h"

using namespace std;

void WptsNavigate::run() {
    if(drone->getState() == States::Armed || drone->getState() == States::Takingoff) {
        drone->TOLService(true);
    }
    else if(drone->getState() == States::Autonomous) {
        if(drone->wptsList.size()>0) {
            drone->executeMission();
        }
        else {
            drone->TOLService(false);
        }
    }
}

WptsNavigate::WptsNavigate(ros::NodeHandle &nh):nh(nh) {
    nh.param("/swarmsim_example/droneType", droneType, 0);
    drone = new Drone(0, nh, droneType);
    drone->arm(true);

    this->wptSub = this->nh.subscribe("/local_way_points", 10, &WptsNavigate::localGoalCB, this);
    takenOff = false;

}

void WptsNavigate::localGoalCB(const geometry_msgs::PoseArrayConstPtr &msg) {
    std::cout<<"Recieved " <<drone->wptsList.size()<<std::endl;
    geometry_msgs::PoseArray pArr = *msg;
    if(drone->wptsList.size() == 0) {
        vector<geometry_msgs::Pose> selectedP = preProcessWpts(pArr);
        drone->addWaypoints(selectedP);
        std::cout<<"added waypoints: "<<selectedP.size()<<std::endl;
    }
    else {
        std::cout<<"Recieved, but not adding as queue is not empty"<<std::endl;
    }
}

vector<geometry_msgs::Pose> WptsNavigate::preProcessWpts(geometry_msgs::PoseArray p_arr) {
    vector<geometry_msgs::Pose> wpts;
    int nPts = 0;
    p_arr.poses.size() > 5 ? nPts = 5 : nPts = p_arr.poses.size();
    std::cout<<"p_arr "<<p_arr.poses.size()<<std::endl;
    for(int i=0 ; i < nPts; i++) {
        wpts.push_back(p_arr.poses[i]);
    }
    return wpts;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "wpts_node");
    ros::NodeHandle nh;
    WptsNavigate navigate(nh);

    ros::Rate loop_rate(50);

    while(ros::ok()) {
        navigate.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
