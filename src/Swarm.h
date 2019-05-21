#include "ros/ros.h"
#include "tf/tf.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>

class Swarm {
public:
  Swarm(const ros::NodeHandle &n, double frequency, int n_drones);
  void iteration(const ros::TimerEvent &e);
  void run(float frequency);

private:
  ros::NodeHandle node;
  float frequency;
  bool to_requested;
  int n_drones;

  void takeoff(int drone_id);
  void arm(int drone_id, bool arm);
  void poseCallback(int drone_id);
  
};