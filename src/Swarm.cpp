#include <iostream>
#include <ros/console.h>
#include "Swarm.h"
#include "state.h"
#include "utils.h"
#include <thread>
#include <chrono>

using namespace std;

Swarm::Swarm(const ros::NodeHandle &n, double frequency, int n_drones, bool fileLoad)
    : frequency(frequency), n_drones(n_drones), nh(n) {
  state = States::Idle;
  phase = Phases::Planning;
  double maxVel = 4;
  double maxAcc = 5;
  droneTrajSolver = new Solver(n_drones, maxVel, maxAcc, 2, frequency);
  yaml_fpath = "/home/malintha/drone_demo/install/share/swarmsim/launch/traj_data/goals.yaml";
  planningInitialized = false;
  optimizingInitialized = false;
  for (int i=0;i<n_drones;i++) {
    Drone* drone = new Drone(i, nh);
    dronesList.push_back(drone); 
  }

  //loading the full trajectories from files 
  if(fileLoad) {
    vector<Trajectory> trajectories = simutils::loadTrajectoriesFromFile(n_drones, nh, true);
    for(int i=0;i<n_drones;i++) {
      Trajectory traj = trajectories[i];
      dronesList[i]->pushTrajectory(traj);
    }
  }

  //performing online trajectory optimization
  else {
    vector<Trajectory> trajectories = getTrajectories(1);
    for(int i=0;i<n_drones;i++) {
      dronesList[i]->pushTrajectory(trajectories[i]);
    }
    horizonLen = trajectories[0].pos.size();
  }
}

void Swarm::iteration(const ros::TimerEvent &e) {
  switch (state)
  {
  case States::Idle:
    checkSwarmForStates(States::Ready);
    break;

  case States::Ready:
    armDrones(true);
    checkSwarmForStates(States::Armed);
    break;

  case States::Armed:
    TOLService(true);
    checkSwarmForStates(States::Autonomous);
    break;

  case States::Autonomous:
    // performPhaseTasks();
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
  //fixme: states don't change without the thread_sleep
  std::this_thread::sleep_for (std::chrono::nanoseconds(1));
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
  int execPointer;
  for(int i=0;i<n_drones;i++) {
    execPointer = this->dronesList[i]->executeTrajectory();
  }
  setSwarmPhase(execPointer);
}

/**
 * todo: change these ratios if want to use receding horizon planning.
 * eg: plan again when progress is 0.5
*/
void Swarm::setSwarmPhase(int execPointer) {
  double progress = execPointer/horizonLen;
  if(progress < 0.6) {
    if(progress == 0) {
      planningInitialized = false;
      optimizingInitialized = false;
      executionInitialized = false;
    }
    phase = Phases::Planning;
  }
  else if(progress < 0.8) {
    phase = Phases::Optimization;
  }
  else {
    phase = Phases::Execution;
  }
}

void Swarm::performPhaseTasks() {
  if(phase == Phases::Planning && !planningInitialized) {
    //initialize the external opertaions such as slam or shape convergence
    //in this case we only load the waypoints from the yaml file
    
    planningInitialized = true;
  }
  else if(phase == Phases::Optimization && !optimizingInitialized) {
    //get next wpts from the planning future and attach them to the swarm
    //initialize the trajectory optimization

    optimizingInitialized = true;
  }
  else if(phase == Phases::Execution && !executionInitialized) {
    //get the optimized trajectories from optimization future and push them to the drones
    
    executionInitialized = true;
  }
}

std::vector<Trajectory> Swarm::getTrajectories(int trajecoryId) {
    wpts = simutils::getTrajectoryList(yaml_fpath, trajecoryId);
    return droneTrajSolver->solve(wpts);
}

void Swarm::setWaypoints(vector<Trajectory> wpts, vector<double> tList) {
  if (phase == Phases::Planning) {
    this->wpts = wpts;
    this->tList = tList;
  }
  else {
    ROS_DEBUG_STREAM("Swarm is not in the planning phase. Waypoints rejected");
  }
}
