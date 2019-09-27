#include <iostream>
#include "swarmsim/Drone.h"
#include "swarmsim/Trajectory.h"

using namespace std;

class WptsNavigate {
    public:
    WptsNavigate();

    private:
    void localGoalCB(const geometry_msgs::PoseArrayConstPtr &msg);
    
};