#include <iostream>
#include "ros/ros.h"
#include "tf/tf.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include "Drone.h"
#include "Trajectory.h"
#include <future>
#include "SimplePlanningPhase.h"

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
bool fileLoad;
  int nHorizons; 
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
  int horizonLen;
  PlanningPhase* planningPhase;
  SimplePlanningPhase spp;
  bool planningInitialized;
  bool optimizingInitialized;
  bool executionInitialized;
  int horizonId;

  /**
   * check the swarm for a given state.
   * The swarm is in a state if all the drones are in the same state
  */
  void checkSwarmForStates(int state);
  void setState(int state);
  void armDrones(bool arm);
  void TOLService(bool takeoff);
  void sendPositionSetPoints();
  //calculates the swarm phase for the swarm based on the horizon length and current time
  void setSwarmPhase(int execPointer);
  void performPhaseTasks();
};