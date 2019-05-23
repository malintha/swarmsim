#include <iostream>
#include <ros/console.h>
#include "Swarm.h"
#include "state.h"

using namespace std;

Swarm::Swarm(const ros::NodeHandle &n, double frequency, int n_drones)
    : frequency(frequency), n_drones(n_drones), nh(n) {
  state = States::Idle;   
  for (int i=0;i<n_drones;i++) {
    Drone* drone = new Drone(i, nh);
    dronesList.push_back(drone); 
  }
}

void Swarm::iteration(const ros::TimerEvent &e) {

  switch (state)
  {
  case States::Idle:
    checkSwarmForStates(States::Ready);
    break;

  case States::Ready:
    ROS_DEBUG("SWARM READY");
    armDrones(true);
    checkSwarmForStates(States::Armed);
    break;

  case States::Armed:
    takeOff();
    checkSwarmForStates(States::Autonomous);
    break;

  case States::Autonomous:
  
    break;

  default:
    break;
  }

}

void Swarm::run(float frequency) {
  this->frequency = frequency;
  ros::Timer timer = nh.createTimer(ros::Duration(1 / frequency),
                                      &Swarm::iteration, this);
  ros::spin();
}

void Swarm::setState(int state) {
  this->state = state;
  ROS_DEBUG_STREAM("Set swarm state: "<<state);
}

void Swarm::checkSwarmForStates(int state) {
  bool swarmInState;
  for(int i = 0; i < n_drones; i++) {
  bool swarmInStateTemp;
  this->dronesList[i]->getState() == state ? swarmInStateTemp = true : swarmInStateTemp = false;
  swarmInState = swarmInState && swarmInStateTemp;
}
  if(swarmInState) {
    setState(state);
  }
}

void Swarm::armDrones(bool arm) {
  for(int i=0;i<n_drones;i++) {
    this->dronesList[i]->arm(arm);
  }
}

void Swarm::takeOff() {
    for(int i=0;i<n_drones;i++) {
    this->dronesList[i]->takeoff();
  }
}
