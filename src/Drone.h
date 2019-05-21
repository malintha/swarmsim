#include <iostream>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "state.h"

using namespace Eigen;

class Drone {
public:
  Drone(int id, const ros::NodeHandle &n);

    /**
     * function for arming or disarming a drone. Uses the service:
     * /<drone_id>/mavros/cmd/arming
     */
    void arm(bool arm);

    /**
     * function for taking off a drone. Uses the service:
     * /<drone_id>/mavros/cmd/takeoff
     */
    void takeoff();
    int getState();

private:
    int id;
    Vector3d curr_pos_local;
    Vector3d curr_pos_global; 
    Vector3d init_pos_global;
    Vector3d init_pos_local;
    bool setInitPosGlobal;
    bool setInitPosLocal;
    float yaw;
    int state;

    ros::NodeHandle nh;
    ros::Subscriber localPositionSub;
    ros::Subscriber globalPositionSub;
    ros::Subscriber poseSub;

    void positionGlobalCB(const sensor_msgs::NavSatFixConstPtr& msg);
    void positionLocalCB(const nav_msgs::OdometryConstPtr& msg);
    void poseCB(const geometry_msgs::PoseStampedConstPtr& msg);
    void setState(int state);
    void ready(bool setInitPosGlobal, bool setInitPosLocal);

// todo: move to a util class
    /**
     * get rpy from geometry_msgs::Quaternion
     */
    static Vector3d getRPY(geometry_msgs::Quaternion orientation);
    
    /**
     * construct the global position cb topic name given the global or local
     * inertial frame
     * /<id>/mavros/global_position/local
     * /<id>/mavros/global_position/global
     */
    std::string getPositionTopic(std::string locale);
    
    /**
     * construct the pose topic name
     * /<id>/mavros/local_position/pose
     */
    std::string getPoseTopic();
};