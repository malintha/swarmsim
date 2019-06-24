#include "PlanningPhase.h"
#include<ros/console.h>

PlanningPhase::PlanningPhase() {}

PlanningPhase::PlanningPhase(int nDrones, double frequency) : nDrones(nDrones), dt(1/frequency) {
    maxVelocity = 4;
    maxAcceleration = 5;
    nChecks = 2;
    doneInitPlanning = false;
    solver = new Solver(nDrones, maxVelocity, maxAcceleration, nChecks, frequency);
}

vector<Trajectory> PlanningPhase::computeSmoothTrajectories() {
    vector<Trajectory> results = solver->solve(discreteWpts);
    return results;
}

vector<Trajectory> PlanningPhase::getPlanningResults() {
    //wait for the initial planning to finish
    do {
        ROS_DEBUG_ONCE("Waiting for initial planning to finish");
    }
    while(!doneInitPlanning); 
    planning_t->join();
    vector<Trajectory> results = fut.get();
    return results;
}

void PlanningPhase::doPlanning(int horizonId) {
    cout<<"in doPlanning base";
}
vector<Trajectory> PlanningPhase::getDiscretePlan(int horizonId) {}

void PlanningPhase::testf() {}