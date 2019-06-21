#include "PlanningPhase.h"

PlanningPhase::PlanningPhase(int nDrones, double frequency) : nDrones(nDrones), dt(1/frequency) {
    maxVelocity = 4;
    maxAcceleration = 5;
    nChecks = 2;
    solver = new Solver(nDrones, maxVelocity, maxAcceleration, nChecks, frequency);
}

vector<Trajectory> PlanningPhase::computeSmoothTrajectories() {
    solver->solve(discreteWpts);
}