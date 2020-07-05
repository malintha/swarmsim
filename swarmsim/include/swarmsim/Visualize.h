#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include "Trajectory.h"
#include "Obstacle.h"
#include <geometry_msgs/Point.h>

using namespace std;
using namespace Eigen;

class Visualize {
    public:
        Visualize(ros::NodeHandle nh, string worldframe, int ndrones, string obstacleConfigFilePath);
        void addToPaths(vector<Trajectory> trajs);
        void draw();

    private:
        int ndrones;
        ros::NodeHandle nh;
        string worldframe;
        ros::Publisher markerPub_traj;
        ros::Publisher markerPub_obs;
        vector<visualization_msgs::Marker> marker_traj;
        vector<visualization_msgs::Marker> marker_obs;
        string obstacleConfigFilePath;
        std::vector<Obstacle> obstacles;
        void initMarkers();
        std::vector<Obstacle> readObstacleConfig();
        void populateObstacles();

        void drawGrid(std::vector<geometry_msgs::Point>);
        void drawPaths(std::vector<Trajectory>);
        void drawAssignments();
        void drawSampling(std::vector<geometry_msgs::Point>);

};
