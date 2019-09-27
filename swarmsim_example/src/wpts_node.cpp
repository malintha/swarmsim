#include <ros/ros.h>
#include "eigen3/Eigen/Dense"
#include "dji_sdk/dji_sdk.h"
#include "swarmsim/Drone.h"
#include "geometry_msgs/PoseArray.h"

using namespace std;

class WptsNavigate {

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_flight_control_node");
    ros::NodeHandle nh;


}

void localGoalCB(const geometry_msgs::PoseArrayConstPtr &msg) {

}