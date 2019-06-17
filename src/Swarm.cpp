#include <iostream>
#include <ros/console.h>
#include "Swarm.h"
#include "state.h"
#include "utils.h"

using namespace std;

Swarm::Swarm(const ros::NodeHandle &n, double frequency, int n_drones, bool fileLoad)
    : frequency(frequency), n_drones(n_drones), nh(n) {
  state = States::Idle;
  double maxVel = 4;
  double maxAcc = 5;
  droneTrajSolver = new Solver(n_drones, maxVel, maxAcc, 2, frequency);
  for (int i=0;i<n_drones;i++) {
    Drone* drone = new Drone(i, nh);
    dronesList.push_back(drone); 
  }

  //loading the full trajectoryies from files 
  if(fileLoad) {
    trajectories = simutils::loadTrajectoriesFromFile(n_drones, nh, true);
    for(int i=0;i<n_drones;i++) {
      Trajectory traj = trajectories[i];
      dronesList[i]->setTrajectory(traj);
    }
  }

  //loading just sub goal positions from files
  else {
    vector<double> tList = simutils::loadTimesFromFile(nh);
    vector<Trajectory> droneWpts = simutils::loadTrajectoriesFromFile(n_drones, nh, false);
    ROS_DEBUG_STREAM("wpts: "<<droneWpts[0].pos.size());

    trajectories = droneTrajSolver->solve(droneWpts, tList);
    for(int i=0;i<n_drones;i++) {
      dronesList[i]->setTrajectory(trajectories[i]);
    }
  }
}

void Swarm::iteration(const ros::TimerEvent &e) {
  switch (state)
  {
  case States::Idle:
    checkSwarmForStates(States::Ready);
    break;

  case States::Ready:
    ROS_DEBUG("Swarm Ready");
    armDrones(true);
    checkSwarmForStates(States::Armed);
    break;

  case States::Armed:
    TOLService(true);
    checkSwarmForStates(States::Autonomous);
    break;

  case States::Autonomous:
    sendPositionSetPoints();
    checkSwarmForStates(States::Reached);
    break;

  case States::Reached:
    TOLService(false);
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

void Swarm::TOLService(bool takeoff) {
    for(int i=0;i<n_drones;i++) {
    this->dronesList[i]->TOLService(takeoff);
  }
}

void Swarm::sendPositionSetPoints() {
  for(int i=0;i<n_drones;i++) {
    this->dronesList[i]->executeTrajectory();
  }
}

