#include "ros/ros.h"
#include "tf/tf.h"
#include <iostream>

using namespace std;

class swarmSim {
public:
  swarmSim(const ros::NodeHandle &n, double frequency): frequency(frequency) {
      node = n;
  }

  void iteration(const ros::TimerEvent &e) {
      std::cout << "swarmsim running" <<std::endl;
  }

  void run(float frequency) {
    this->frequency = frequency;
    ros::Timer timer = node.createTimer(ros::Duration(1 / frequency),
                                        &swarmSim::iteration, this);
    ros::spin();
  }

private:
  ros::NodeHandle node;
  float frequency;
    
};

int main(int argc, char **argv) {
  float frequency = 10;
  ros::init(argc, argv, "swarmsim");
  static ros::NodeHandle n("~");
  swarmSim simulator(n, frequency);
  simulator.run(frequency);
  return 0;
}
