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

  //the swarm is ready when all the drones are ready
  case States::Ready:
    armDrones();
    checkSwarmForStates(States::Armed);
    break;

  case States::Armed:
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
}

void Swarm::checkSwarmForStates(int state) {
  bool swarmInState;
  for(int i = 0; i < n_drones; i++) {
  bool swarmInStateTemp;
  dronesList[i]->getState() == state ? swarmInStateTemp = true : swarmInStateTemp = false;
  swarmInState = swarmInState && swarmInStateTemp;
}
if(swarmInState) {
  setState(state);
}
}

void Swarm::armDrones() {
  for(int i=0;i<n_drones;i++) {
    dronesList[i]->arm(true);
  }
}

// void Swarm::checkSwarmReady() {
//   bool swarmReady;
//   for(int i = 0; i < n_drones; i++) {
//     bool swarmReadyTemp;
//     dronesList[i]->getState() == States::Ready ? swarmReadyTemp = true : swarmReadyTemp = false;
//     swarmReady = swarmReady && swarmReadyTemp;
//   }
//   if(swarmReady) {
//     setState(States::Ready);
//   }
// }


