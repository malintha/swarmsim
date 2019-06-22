#include "PlanningPhase.h"

PlanningPhase::PlanningPhase(int nDrones, double frequency) : nDrones(nDrones), dt(1/frequency) {
    maxVelocity = 4;
    maxAcceleration = 5;
    nChecks = 2;
    doneInitPlanning = false;
    solver = new Solver(nDrones, maxVelocity, maxAcceleration, nChecks, frequency);
}

vector<Trajectory> PlanningPhase::computeSmoothTrajectories() {
    solver->solve(discreteWpts);
}

vector<Trajectory> PlanningPhase::getPlanningResults() {
    //handle the exception when t isn't finished.
    // planning_t->join();
    vector<Trajectory> results = fut.get();
    cout<<"retrieved planning data"<<results[0].pos.size()<<endl;
    return results;
}

void PlanningPhase::doPlanning(int horizonId) {}

vector<Trajectory> PlanningPhase::getDiscretePlan(int horizonId) {}