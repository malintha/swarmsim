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

    auto doPlanningExpr = [horizonId, this, sharedP]() {
        discreteWpts = this->getDiscretePlan(horizonId);
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
};

vector<Trajectory> SimplePlanningPhase::getDiscretePlan(int horizonId) {
    ROS_DEBUG_STREAM("yaml file path: "<<yamlFpath);
    char cstr[yamlFpath.size()+1];
    copy(yamlFpath.begin(), yamlFpath.end(), cstr);
    cstr[yamlFpath.size()] = '\0';
    return simutils::processYamlFile(cstr, horizonId);
}

void SimplePlanningPhase::testf() {
    cout<<"child class"<<endl;
}