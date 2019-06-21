#include "PlanningPhase.h"

/**
 * This implementation loads the discrete waypoints from a yaml file instead of doing
 * actual planning. And then calculate the smooth trajectories.
*/
class SimplePlanningPhase : PlanningPhase {
    public:
        char* yamlFpath;
        SimplePlanningPhase(int nDrones, double frequency, string yamlFpath);
        void doPlanning(promise<vector<Trajectory> > &p, int horizonId) override;
        
        /**
         * Returns the discrete waypoints from the yaml file
        */
        vector<Trajectory> getDiscretePlan(int horizonId) override;
};