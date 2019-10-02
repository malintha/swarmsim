#include <iostream>
#include "swarmsim/Drone.h"
#include "swarmsim/Trajectory.h"
#include "geometry_msgs/PoseArray.h"

using namespace std;

class WptsNavigate {
    public:
    int droneType;
    ros::NodeHandle nh;
    WptsNavigate(ros::NodeHandle &nh);
    void run();
    ros::Subscriber wptSub;
    private:
    void localGoalCB(const geometry_msgs::PoseArrayConstPtr &msg);
    vector<geometry_msgs::Pose> preProcessWpts(geometry_msgs::PoseArray p_arr);
    Drone* drone;
    bool takenOff;

};