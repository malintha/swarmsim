#include <iostream>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "state.h"
#include "Trajectory.h"
#include "mavros_msgs/State.h"
#include "gazebo_msgs/ModelStates.h"
#include "ExternalAPI.h"
#include "geometry_msgs/Pose.h"
#include "list"

using namespace Eigen;

class Drone {
public:
  Drone(int id, const ros::NodeHandle &n, int droneType);

    /**
     * function for arming or disarming a drone. Uses the service:
     * /<drone_id>/mavros/cmd/arming
     */
    void arm(bool arm);

    /**
     * function for taking off a drone. Uses the service:
     * /<drone_id>/mavros/cmd/takeoff
     */
    void TOLService(bool takeoff);
    int getState();
    void setState(int state);

    void setTrajectory(Trajectory trajectory);
    int executeTrajectory();
    void pushTrajectory(Trajectory trajectory);
    /**
     * Perform the conversion between the droneLocal frame and the gazebo frame.
     * MavRos expects the waypoints in the drone local frame, whereas the yaml specifies 
     * the waypoints in the gazebo frame.
     */
    Vector3d getLocalWaypoint(Vector3d waypoint);

    void executeMission();
    void setCurrentSetPoint();
    list<geometry_msgs::Pose> wptsList;
    void addWaypoints(vector<geometry_msgs::Pose> newWpts);


private:
    int id;
    Vector3d curr_pos_local;
    Vector3d curr_pos_global; 
    Vector3d init_pos_global;
    Vector3d init_pos_local;
    float yaw;
    int state;
    float takeoffHeight;
    Trajectory trajectory;
    int execPointer;
    bool setReady;
    Vector3d initGazeboPos;
    int gazeboElementIdx;
    ExternalAPI *extAPI;
    int apiType;

    std::vector<Trajectory> TrajectoryList;
    int trajectoryId;

    ros::NodeHandle nh;
    ros::Subscriber localPositionSub;
    ros::Subscriber globalPositionSub;
    ros::Subscriber poseSub;
    ros::Publisher posSetPointPub;
    ros::Subscriber mavrosStateSub;
    ros::Subscriber gazeboStateSub;

    bool reachedGoal(geometry_msgs::PoseStamped setPoint);

    //for the waypoints following
    geometry_msgs::Pose currentTarget;
    bool wptsFollowing;
};