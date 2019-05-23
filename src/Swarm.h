#include <iostream>
#include "ros/ros.h"
#include "tf/tf.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include "Drone.h"

class Swarm {
public:
  Swarm(const ros::NodeHandle &n, double frequency, int n_drones);
  void iteration(const ros::TimerEvent &e);
  void run(float frequency);

private:
  int state; 
  ros::NodeHandle nh;
  float frequency;
  int n_drones;
  std::vector<Drone*> dronesList;
  /**
   * check the swarm for a given state.
   * The swarm is in a state if all the drones are in the same state
  */
  void checkSwarmForStates(int state);
  void setState(int state);
  void armDrones(bool arm);
  void takeOff();
  void sendPositionSetPoints();
};