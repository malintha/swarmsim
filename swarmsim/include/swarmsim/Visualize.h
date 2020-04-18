#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include "Trajectory.h"

using namespace std;
using namespace Eigen;

class Visualize {
    public:
        Visualize(ros::NodeHandle nh, string worldframe, int ndrones);
        void addToPaths(vector<Trajectory> trajs);
        void draw();

    private:
        int ndrones;
        ros::NodeHandle nh;
        string worldframe;
        ros::Publisher markerPub;
        vector<visualization_msgs::Marker> markerVec;
        void initPaths();
};
