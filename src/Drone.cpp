#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"

#include "ros/console.h"
#include "Drone.h"


using namespace std;

Drone::Drone(int id, const ros::NodeHandle &n) : id(id), nh(n) {
  ROS_DEBUG_STREAM("Initializing drone "<<id);
  setState(States::Idle);
  std::string globalPositionTopic = getPositionTopic("global");
  std::string localPositionTopic = getPositionTopic("local");
  std::string poseTopic = getPoseTopic();

  localPositionSub = nh.subscribe(localPositionTopic, 10, &Drone::positionLocalCB, this);
  globalPositionSub = nh.subscribe(globalPositionTopic, 10, &Drone::positionGlobalCB, this);

}

int Drone::getState() {
  return state;
}

void Drone::positionLocalCB(const nav_msgs::Odometry::ConstPtr &msg) {
  geometry_msgs::Point pos = msg->pose.pose.position;
  curr_pos_local << pos.x, pos.y, pos.z;
  yaw = getRPY(msg->pose.pose.orientation)[2];
  if(!setInitPosLocal) {
    init_pos_local = curr_pos_local;
    setInitPosLocal = true;
    ROS_DEBUG_STREAM("Drone: "<<id<<" Init position local: "<<init_pos_local[0]<<" "<<init_pos_local[1]<<" "<<init_pos_local[2]);
    ready(setInitPosGlobal, setInitPosLocal);
  }
}

void Drone::positionGlobalCB(const sensor_msgs::NavSatFixConstPtr &msg) {
  curr_pos_global << msg->latitude, msg->longitude, msg->altitude;
  if(!setInitPosGlobal) {
    init_pos_global = curr_pos_global;
    setInitPosGlobal = true;
    ROS_DEBUG_STREAM("Drone: "<<id<<" Init position global: "<<init_pos_global[0]<<" "<<init_pos_global[1]<<" "<<init_pos_global[2]);
    ready(setInitPosGlobal, setInitPosLocal);
  }
}

void Drone::poseCB(const geometry_msgs::PoseStampedConstPtr &msg) {}

void Drone::arm(bool arm) {
  stringstream ss_arm;
  ss_arm << "/" << id << "/mavros/cmd/arming";
  string arm_service = ss_arm.str();
  ros::ServiceClient arming_cl =
      nh.serviceClient<mavros_msgs::CommandBool>(arm_service);

  mavros_msgs::CommandBool srv;
  srv.request.value = arm;
  ROS_DEBUG_STREAM("Waiting for arm service " << arm_service);
  arming_cl.waitForExistence();
  if (arming_cl.call(srv)) {
    ROS_DEBUG_STREAM("Arm request sent");
  } else {
    ROS_ERROR_STREAM("Arm request failed for drone: "<<id);
  }
  arm == true ? state = States::Armed : States::Idle;

}

void Drone::takeoff() {
  stringstream ss_to;
  ss_to << "/" << id << "/mavros/cmd/takeoff";
  string to_service = ss_to.str();
  ros::ServiceClient to_cl =
      nh.serviceClient<mavros_msgs::CommandTOL>(to_service);

  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.latitude = init_pos_global[0];
  srv_takeoff.request.longitude = init_pos_global[1];
  srv_takeoff.request.altitude = init_pos_global[2] + 2.6;
  srv_takeoff.request.min_pitch = 0;
  srv_takeoff.request.yaw = M_PI/2;

  ROS_DEBUG_STREAM("Drone: "<<id<<" taking off at "<< init_pos_global[0]<<" "<<init_pos_global[1]<<" "<<init_pos_global[2]);

  ROS_DEBUG_STREAM("Waiting for takeoff service " << to_service);
  to_cl.waitForExistence();
  if (to_cl.call(srv_takeoff)) {
    ROS_DEBUG_STREAM("Takeoff request sent" << to_service);
  }
  else {
    ROS_ERROR_STREAM("Takeoff request failed for drone "<<id);  
  }
  state = States::Autonomous;
}


std::string Drone::getPositionTopic(std::string locale) {
  std::stringstream ss;
  ss << "/" << this->id << "/mavros/global_position/" << locale;
  return ss.str();
}


std::string Drone::getPoseTopic() {
  std::stringstream ss;
  ss << "/" << this->id << "/mavros/local_position/pose";
  return ss.str();
}


Vector3d Drone::getRPY(geometry_msgs::Quaternion orientation) {
  tfScalar roll, pitch, yaw;
  Vector3d rpy;
  tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z,
                               orientation.w))
      .getRPY(roll, pitch, yaw);
  rpy << roll, pitch, yaw;
  return rpy;
}

void Drone::setState(int state) {
  this->state = state;
  ROS_DEBUG_STREAM("Drone: "<<id<<" Set drone state "<<state);
}

void Drone::ready(bool setInitPosGlobal, bool setInitPosLocal) {
  if(setInitPosGlobal && setInitPosLocal) {
    setState(States::Ready);
  }
}