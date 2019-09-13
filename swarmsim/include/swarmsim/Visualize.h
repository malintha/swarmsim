#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "Trajectory.h"
#include "ros/console.h"

using namespace std;

class Visualize {
    public:
    Visualize(int nDrones);
    void draw(vector<Trajectory> trs);

    private:
    ros::Publisher marker_pub;
    visualization_msgs::Marker comMarker;
    vector<visualization_msgs::Marker> markerVec;

};