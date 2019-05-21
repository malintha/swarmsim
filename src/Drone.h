#include <iostream>
#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"

#ifndef state
#define state
enum State { Idle, TakingOff, Landing, Autonomous };
#endif

class Drone {
public:
  Drone(int id, const ros::NodeHandle &n);

private:
    int id;
    geometry_msgs::PoseStamped pose;
    ros::NodeHandle nh;
    ros::Subscriber localPositionSub;
    ros::Subscriber globalPositionSub;
    ros::Subscriber poseSub;

    void arm(bool arm);
    void takeoff();
    void positionGlobalCB(const sensor_msgs::NavSatFixConstPtr& msg);
    void positionLocalCB(const nav_msgs::OdometryConstPtr& msg);
    void poseCB(const geometry_msgs::PoseStampedConstPtr& msg);

    std::string getPositionTopic(std::string locale);
    std::string getPoseTopic();
};