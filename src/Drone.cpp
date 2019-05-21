#include "Drone.h"

Drone::Drone(int id,const ros::NodeHandle &n):id(id),nh(n) {
    std::string globalPositionTopic = getPositionTopic("global");
    std::string localPositionTopic = getPositionTopic("local");
    std::string poseTopic = getPoseTopic();

    // localPositionSub = nh.subscribe(localPositionTopic,1,&positionLocalCB);
}

void Drone::positionLocalCB(const nav_msgs::OdometryConstPtr& msg) {
    
}

void Drone::positionGlobalCB(const sensor_msgs::NavSatFixConstPtr& msg) {
    
}

void Drone::poseCB(const geometry_msgs::PoseStampedConstPtr& msg) {
    
}
/**
 * construct the global position cb topic name given the global or local inertial frame
 * /<id>/mavros/global_position/local
 * /<id>/mavros/global_position/global
*/
std::string Drone::getPositionTopic(std::string locale) {
    std::stringstream ss;
    ss << "/"<< this->id<< "/mavros/global_position/"<<locale;
    return ss.str();
}

/**
 * construct the pose topic name
 * /<id>/mavros/local_position/pose
*/
std::string Drone::getPoseTopic(){
    std::stringstream ss;
    ss << "/"<< this->id<< "/mavros/local_position/pose";
    return ss.str();
}