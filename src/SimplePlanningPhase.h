#include "PlanningPhase.h"

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
        /**
         * Returns the discrete waypoints from the yaml file
        */
        vector<Trajectory> getDiscretePlan(int horizonId) override;
        // vector<Trajectory> getPlanningResults() override;
};