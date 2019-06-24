#include <algorithm>
#include "SimplePlanningPhase.h"
#include "utils.h"
#include <exception>

SimplePlanningPhase:: SimplePlanningPhase(int nDrones, double frequency, string yamlFpath) : PlanningPhase(nDrones, frequency) {
    this->yamlFpath = yamlFpath; 
};

void SimplePlanningPhase:: doPlanning(int horizonId) {
    auto sharedP = make_shared<promise<vector<Trajectory> > >();
    fut = sharedP->get_future();
    auto doPlanningExpr = [horizonId, this, sharedP]() {
        discreteWpts = this->getDiscretePlan(horizonId);
        vector<Trajectory> smoothTrajs = computeSmoothTrajectories();
        try {
            sharedP->set_value_at_thread_exit(smoothTrajs);
        }
        catch(exception& e) {
            ROS_ERROR_STREAM("Caught a future_error \"" << e.what());
        }
        doneInitPlanning = true;
    };
    planning_t = new thread(doPlanningExpr);
};

vector<Trajectory> SimplePlanningPhase::getDiscretePlan(int horizonId) {
    ROS_DEBUG_STREAM("yaml file path: "<<yamlFpath);
    char cstr[yamlFpath.size()+1];
    copy(yamlFpath.begin(), yamlFpath.end(), cstr);
    cstr[yamlFpath.size()] = '\0';
    return simutils::processYamlFile(cstr, horizonId);
}

// vector<Trajectory> SimplePlanningPhase::getPlanningResults() {
//     //wait for the initial planning to finish
//     do {}
//     while(!doneInitPlanning);
//     planning_t->join();
//     vector<Trajectory> results = fut.get();
//     return results;
// }