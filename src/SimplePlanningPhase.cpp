#include <algorithm>
#include "SimplePlanningPhase.h"
#include "utils.h"

SimplePlanningPhase::SimplePlanningPhase() {};

SimplePlanningPhase:: SimplePlanningPhase(int nDrones, double frequency, string yamlFpath) : PlanningPhase(nDrones, frequency) {
    this->yamlFpath = yamlFpath;
};

void SimplePlanningPhase:: doPlanning(int horizonId) {
    auto sharedP = make_shared<promise<vector<Trajectory> > >();
    fut = sharedP->get_future();
    // auto threadExcetionPtr = make_shared<exception_ptr>();
    auto doPlanningExpr = [horizonId, this, sharedP]() {
        try {
            discreteWpts = this->getDiscretePlan(horizonId);
        }
        catch(range_error& e) {
            ROS_ERROR_STREAM("Error occurred while planning!");
            return;
        }
        bool initialQP;
        horizonId == 0 ? initialQP = true : initialQP = false;
        vector<Trajectory> smoothTrajs = computeSmoothTrajectories(initialQP);
        try {
            sharedP->set_value_at_thread_exit(smoothTrajs);
        }
        catch(std::future_error& e) {
            ROS_ERROR_STREAM("Caught a future_error while fulfilling the promise\"" << e.what());
        }
        doneInitPlanning = true;
    };
    planning_t = new thread(doPlanningExpr);
    return;
};

vector<Trajectory> SimplePlanningPhase::getDiscretePlan(int horizonId) {
    ROS_DEBUG_STREAM("yaml file path: "<<yamlFpath);
    char cstr[yamlFpath.size()+1];
    copy(yamlFpath.begin(), yamlFpath.end(), cstr);
    cstr[yamlFpath.size()] = '\0';
    vector<Trajectory> planningResults;
    try {
        planningResults = simutils::processYamlFile(cstr, horizonId);
    }
    catch (range_error& e) {
        ROS_ERROR_STREAM(e.what()<<" "<<horizonId);
        throw range_error(e.what());
    }
    return planningResults; 
}