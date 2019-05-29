#include <iostream>
#include <fstream>
#include <ros/console.h>
#include "Swarm.h"
#include "state.h"

using namespace std;

Swarm::Swarm(const ros::NodeHandle &n, double frequency, int n_drones, bool fileLoad)
    : frequency(frequency), n_drones(n_drones), nh(n) {
  state = States::Idle;   
  for (int i=0;i<n_drones;i++) {
    Drone* drone = new Drone(i, nh);
    dronesList.push_back(drone); 
  }
  if(fileLoad) {
    trajectories = loadTrajectoriesFromFile();
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

std::vector<Trajectory> Swarm::loadTrajectoriesFromFile(){
  std::vector<Trajectory> trajList;
  std::string filePath;
  if(nh.getParam("/swarmsim/trajDir",filePath)) {
    for(int i=0;i<n_drones;i++) {
      std::stringstream ss;
      ss << filePath <<"pos_"<<i<<".txt";
      ROS_DEBUG_STREAM("Loading trajectories from file "<<ss.str());
      std::ifstream posStream(ss.str());
      std::string posLine;
      double x, y, z;
      Trajectory traj;
      while(posStream >> x >> y >> z) {
        Eigen::Vector3d pos;
        pos << x, y, z;
        traj.position.push_back(pos);
      }
      trajList.push_back(traj);
      dronesList[i]->setTrajectory(traj);
    }
  }
ROS_DEBUG_STREAM("Trajectories loaded from file");
}