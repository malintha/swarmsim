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
        void addToGrid(std::vector<geometry_msgs::Point>);
        void addStartGoal(std::vector<geometry_msgs::Point>);
        void addAgentPaths(std::vector<std::vector<geometry_msgs::Point>>);
        void addToTopo(std::vector<geometry_msgs::Point>);


    private:
        int ndrones;
        int agents;
        ros::NodeHandle nh;
        string worldframe;
        ros::Publisher markerPub_traj;
        ros::Publisher markerPub_obs;
        ros::Publisher markerPub_samples;
        ros::Publisher markerPub_paths;
        ros::Publisher markerPub_topo;

        vector<visualization_msgs::Marker> marker_traj;
        vector<visualization_msgs::Marker> marker_obs;
        vector<visualization_msgs::Marker> maker_agentPaths;
        visualization_msgs::Marker gridMarker;
        visualization_msgs::Marker startMarker;
        visualization_msgs::Marker topoMarker;


        string obstacleConfigFilePath;
        std::vector<Obstacle> obstacles;

        void initMarkers();
        std::vector<Obstacle> readObstacleConfig();
        void populateObstacles();

        // void addToPaths(std::vector<Trajectory>);
        void drawAssignments();

};
