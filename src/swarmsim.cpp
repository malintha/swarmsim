#include "ros/ros.h"
#include "tf/tf.h"
#include <iostream>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <ros/console.h>
#include "Swarm.h"

#define n_robots 1

using namespace std;

int main(int argc, char **argv) {
  float frequency = 10;
  ros::init(argc, argv, "swarmsim");
  static ros::NodeHandle n("~");
  Swarm simulator(n, frequency, n_robots);
  simulator.run(frequency);
  return 0;
}
