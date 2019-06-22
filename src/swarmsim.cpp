#include "Swarm.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include <iostream>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <ros/console.h>

using namespace std;

int main(int argc, char **argv) {
  float frequency = 10;
  ros::init(argc, argv, "swarmsim");
  static ros::NodeHandle n("~");
  int nDrones;
  bool fileLoad;
  n.param("/swarmsim/nDrones", nDrones, 1);
  n.param("/swarmsim/fileLoad", fileLoad, false);

  ROS_DEBUG_STREAM("Number of drones "<<nDrones);
  Swarm simulator(n, frequency, nDrones, fileLoad);
  // simulator.run(frequency);
  return 0;
}
