#include <algorithm>
#include "SimplePlanningPhase.h"
#include "utils.h"

SimplePlanningPhase:: SimplePlanningPhase(int nDrones, double frequency, string yamlFpath) : PlanningPhase(nDrones, frequency) {
    char cstr[yamlFpath.size()];
    copy(yamlFpath.begin(), yamlFpath.end(), cstr);
    yamlFpath = cstr;
};

void SimplePlanningPhase:: doPlanning(promise<vector<Trajectory> > &p, int horizonId) {
    discreteWpts = this->getDiscretePlan(horizonId);
    vector<Trajectory> smoothTrajs = computeSmoothTrajectories();
    p.set_value(smoothTrajs);
};

vector<Trajectory> SimplePlanningPhase::getDiscretePlan(int horizonId) {
    return simutils::processYamlFile(yamlFpath, horizonId);
}