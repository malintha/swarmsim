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
  ros::NodeHandle nh;
  float frequency;
  bool to_requested;
  int n_drones;
  Drone* drone;


  void poseCallback(int drone_id);
  
};