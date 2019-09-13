#include <iostream>
#include <future>
#include <vector>
#include "Trajectory.h"
#include "solver.h"
#include "DiscretePlanner.h"
#include <thread>
#include "Visualize.h"

using namespace std;

class PlanningPhase {
public:
    //set ndrones, map etc in corresponding derived classes.
    int nHorizons;

    PlanningPhase();
    PlanningPhase(int nDrones, double frequency);

    Visualize* vs;
    bool doneInitPlanning;
    DiscretePlanner *discretePlanner;
    int nDrones;
    double maxVelocity;
    double maxAcceleration;
    double frequency;
    int nChecks;
    vector<Trajectory> discreteWpts;
    thread *planning_t;

    vector<Trajectory> computeSmoothTrajectories();

    future<vector<Trajectory> > fut;

    // override in derived classes
    virtual void doPlanning(int horizonId);

    virtual vector<Trajectory> getDiscretePlan(int horizonId);

    // virtual void computeFormations();
    // virtual void assignGoals();
    virtual vector<Trajectory> getPlanningResults();
};