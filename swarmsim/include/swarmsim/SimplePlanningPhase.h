#include "PlanningPhase.h"
#include "utils.h"
#include "geometry_msgs/PoseArray.h"
#include <ros/ros.h>

/**
 * This implementation loads the discrete waypoints from a yaml file instead of doing
 * actual planning. And then calculate the smooth trajectories.
*/
class SimplePlanningPhase : public PlanningPhase {
    public:
        string yamlFpath;
        promise<vector<Trajectory> > p;
        SimplePlanningPhase();
        SimplePlanningPhase(int nDrones, double frequency, string yamlFpath);
        void doPlanning(int horizonId) override;
        YamlDescriptor yamlDescriptor;
        ros::Subscriber localGoalsSub;
        /**
         * Returns the discrete waypoints from the yaml file
        */
        vector<Trajectory> getDiscretePlan(int horizonId) override;
        void localGoalsCB(const geometry_msgs::PoseArray& msg);
        geometry_msgs::PoseArray path;
        vector<Trajectory> getExecutionTrajectory();

};