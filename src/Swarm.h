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
#include <future>

class Swarm {
public:
  Swarm(const ros::NodeHandle &n, double frequency, int n_drones, bool fileLoad);
  void iteration(const ros::TimerEvent &e);
  void run(float frequency);

/**
 * This function sets wpts and tlist for the next horizon. The idea is to
 * let external parties to set waypoints for the swarm as long as the swarm is
 * in the planning phase.
 */
  void setWaypoints(vector<Trajectory> droneWpts, vector<double> tList); 

private:
  int phase; 
  char* yaml_fpath;
  int state; 
  ros::NodeHandle nh;
  float frequency;
  int n_drones;
  std::vector<Drone*> dronesList;
  //stores the incoming wpts
  std::vector<Trajectory> wpts;
  std::vector<double> tList;
  Solver* droneTrajSolver;
  int horizonLen;
  std::promise<vector<Trajectory> > planningProm;
  std::thread* planningTh;
  /**
   * check the swarm for a given state.
   * The swarm is in a state if all the drones are in the same state
  */
  void checkSwarmForStates(int state);
  void setState(int state);
  void armDrones(bool arm);
  void TOLService(bool takeoff);
  void sendPositionSetPoints();
  //calculates the RHP phase for the swarm based on the Horizon length and current time
  void setSwarmPhase(int execPointer);

  //set the waypoints and tList by reading the yaml file
  void setYamlWaypoints(int trajectoryId);
  bool planningInitialized;
  bool optimizingInitialized;
  bool executionInitialized;
  void performPhaseTasks();



  std::vector<Trajectory> getTrajectories(int trajecoryId);
};