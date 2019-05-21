#include "Swarm.h"
#include <iostream>
#include <ros/console.h>

using namespace std;

Swarm::Swarm(const ros::NodeHandle &n, double frequency, int n_drones)
    : frequency(frequency), n_drones(n_drones) {
  node = n;
  to_requested = false;
}

void Swarm::iteration(const ros::TimerEvent &e) {

  if (!to_requested) {
    this->arm(0,true);
    this->takeoff(0);
  }
  to_requested = true;
}

void Swarm::run(float frequency) {
  this->frequency = frequency;
  ros::Timer timer = node.createTimer(ros::Duration(1 / frequency),
                                      &Swarm::iteration, this);
  ros::spin();
}

/**
 * function for arming or disarming a drone. Uses the service:
 * /<drone_id>/mavros/cmd/arming
 */
void Swarm::arm(int drone_id, bool arm) {
  stringstream ss_arm;
  ss_arm << "/" << drone_id << "/mavros/cmd/arming";
  string arm_service = ss_arm.str();
  ros::ServiceClient arming_cl =
      node.serviceClient<mavros_msgs::CommandBool>(arm_service);

  mavros_msgs::CommandBool srv;
  srv.request.value = arm;
  ROS_DEBUG_STREAM("## wait for service " << arm_service);
  arming_cl.waitForExistence();
  if (arming_cl.call(srv)) {
    ROS_DEBUG_STREAM("ARM send ok");
  } else {
    ROS_ERROR_STREAM("Failed arming or disarming drone "<<drone_id);
  }
}

/**
 * function for taking off a drone. Uses the service:
 * /<drone_id>/mavros/cmd/takeoff
 */
void Swarm::takeoff(int drone_id) {
  stringstream ss_to;
  ss_to << "/" << drone_id << "/mavros/cmd/takeoff";
  string to_service = ss_to.str();
  ros::ServiceClient to_cl =
      node.serviceClient<mavros_msgs::CommandTOL>(to_service);

  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = 2.5;
  srv_takeoff.request.latitude = 37.7332491;
  srv_takeoff.request.longitude = -119.5618833;
  srv_takeoff.request.min_pitch = 0;
  srv_takeoff.request.yaw = M_PI/2;

  ROS_DEBUG_STREAM("## wait for service " << to_service);
  to_cl.waitForExistence();
  if (to_cl.call(srv_takeoff)) {
    ROS_DEBUG_STREAM("## Set to command " << to_service);
  }
  else {
    ROS_ERROR_STREAM("Failed to takeoff drone "<<drone_id);  
  }
}



