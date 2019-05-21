#include "Swarm.h"
#include <iostream>
#include <ros/console.h>

using namespace std;

Swarm::Swarm(const ros::NodeHandle &n, double frequency, int n_drones)
    : frequency(frequency), n_drones(n_drones), nh(n) {

  drone = new Drone(0, nh);
  to_requested = false;
}

void Swarm::iteration(const ros::TimerEvent &e) {
  if(drone->getState() == States::Ready) {
    drone->arm(true);
  }
  if(drone->getState() == States::Armed) {
    drone->takeoff();
  }
}

void Swarm::run(float frequency) {
  this->frequency = frequency;
  ros::Timer timer = nh.createTimer(ros::Duration(1 / frequency),
                                      &Swarm::iteration, this);
  ros::spin();
}





