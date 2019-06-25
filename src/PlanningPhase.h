#include <iostream>
#include <future>
#include <vector>
#include "Trajectory.h"
#include "optimization/solver.h"
#include "DiscretePlanner.h"
#include <thread>

using namespace std;

class PlanningPhase {
    public:
        //set ndrones, map etc in corresponding derived classes.
        PlanningPhase();
        PlanningPhase(int nDrones, double frequency);
        bool doneInitPlanning;
        DiscretePlanner* discretePlanner;
        int nDrones;
        double maxVelocity;
        double maxAcceleration;
        double frequency;
        int nChecks;
        vector<Trajectory> discreteWpts;
        thread* planning_t;
        vector<Trajectory> computeSmoothTrajectories(bool initialQP);
        future<vector<Trajectory> > fut;

        // override in derived classes
        virtual void doPlanning(int horizonId);
        virtual vector<Trajectory> getDiscretePlan(int horizonId);
        // virtual void computeFormations();
        // virtual void assignGoals();
        virtual vector<Trajectory> getPlanningResults();
        virtual void testf();
};