#include <algorithm>
#include "SimplePlanningPhase.h"
#include "utils.h"

SimplePlanningPhase:: SimplePlanningPhase(int nDrones, double frequency, string yamlFpath) : PlanningPhase(nDrones, frequency) {
    this->yamlFpath = yamlFpath; 
    // char cstr[yamlFpath.size()];
    // copy(yamlFpath.begin(), yamlFpath.end(), cstr);
    // yamlFpath = cstr;
};

void SimplePlanningPhase:: doPlanning(int horizonId) {
    auto sharedP = make_shared<promise<vector<Trajectory> > >();
    fut = sharedP->get_future();
    auto doPlanningExpr = [horizonId, this, sharedP]() {
        std::cout<<"do planning_0 "<<yamlFpath<<endl;
        discreteWpts = this->getDiscretePlan(horizonId);
        vector<Trajectory> smoothTrajs = computeSmoothTrajectories();
        sharedP->set_value_at_thread_exit(smoothTrajs);
        doneInitPlanning = true;
    };
    planning_t = new thread(doPlanningExpr);
    std::cout<<"do planning_1"<<endl;

};

vector<Trajectory> SimplePlanningPhase::getDiscretePlan(int horizonId) {
    std::cout<<"yaml file path: "<<yamlFpath<<endl;
    char cstr[yamlFpath.size()+1];
    copy(yamlFpath.begin(), yamlFpath.end(), cstr);
    cstr[yamlFpath.size()] = '\0';
    return simutils::processYamlFile(cstr, horizonId);
}