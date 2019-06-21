#include <iostream>
#include <future>
#include <vector>
#include "Trajectory.h"
#include "optimization/solver.h"
#include "DiscretePlanner.h"

using namespace std;

class PlanningPhase {
    public:
        //set ndrones, map etc in corresponding derived classes.
        PlanningPhase(int nDrones, double frequency);

        Solver* solver;
        DiscretePlanner* discretePlanner;
        int nDrones;
        double maxVelocity;
        double maxAcceleration;
        double dt;
        int nChecks;
        vector<Trajectory> discreteWpts;

        vector<Trajectory> computeSmoothTrajectories();

        // override in derived classes
        virtual void doPlanning(promise<vector<Trajectory> > &p, int horizonId);
        virtual vector<Trajectory> getDiscretePlan(int horizonId);
        virtual void computeFormations();
        virtual void assignGoals();

};