#include "PlanningPhase.h"
#include "utils.h"
#include <geometry_msgs/PoseArray.h>

/**
 * This implementation loads the discrete waypoints from a yaml file instead of doing
 * actual planning. And then calculate the smooth trajectories.
*/
class SimplePlanningPhase : public PlanningPhase {
    public:
        bool recievedPaths;
        ros::NodeHandle nh;
        string yamlFpath;
        promise<vector<Trajectory> > p;
        vector<Trajectory> recievedTrajs;
        int nDrones;
        SimplePlanningPhase();
        SimplePlanningPhase(int nDrones, double frequency, string yamlFpath);
        void doPlanning(int horizonId,std::vector<Trajectory> prevPlan) override;
        YamlDescriptor yamlDescriptor;
        ros::Subscriber robotPathsSub;
        /**
         * Returns the discrete waypoints from the yaml file
        */
        vector<Trajectory> getDiscretePlan(int horizonId) override;
        // vector<Trajectory> getPlanningResults() override;
        void robotPathsCB(const geometry_msgs::PoseArrayConstPtr& msg);
};