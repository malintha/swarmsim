#include <iostream>
#include "ros/ros.h"
#include "tf/tf.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include "Drone.h"
#include "Trajectory.h"
#include "optimization/solver.h"

class Swarm {
public:
  Swarm(const ros::NodeHandle &n, double frequency, int n_drones, bool fileLoad);
  void iteration(const ros::TimerEvent &e);
  void run(float frequency);

private:
  char* yaml_fpath;
  int state; 
  ros::NodeHandle nh;
  float frequency;
  int n_drones;
  std::vector<Drone*> dronesList;
  //stores the trajectories of drones
  std::vector<Trajectory> trajectories;
  Solver* droneTrajSolver;

  /**
   * check the swarm for a given state.
   * The swarm is in a state if all the drones are in the same state
  */
  void checkSwarmForStates(int state);
  void setState(int state);
  void armDrones(bool arm);
  void TOLService(bool takeoff);
  void sendPositionSetPoints();
  std::vector<Trajectory> loadTrajectoriesFromFile(bool subGoalsOnly);
  std::vector<double> loadTimesFromFile();
};