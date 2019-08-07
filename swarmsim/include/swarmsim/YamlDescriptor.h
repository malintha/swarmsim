#ifndef YAML_DESCRIPTOR
#define YAML_DESCRIPTOR

#include <iostream>
#include "Trajectory.h"
#include <vector>

using namespace std;

struct DroneTrajectory {
    vector<Trajectory> droneTrajectory;
};

class YamlDescriptor{
    private:
        int nHorizons;
        int nSubgoals;
        int nDrones;
        vector<vector<double> > timesArray;
        double passThresholdMoving;
        double passThresholdHover;
        vector<DroneTrajectory> droneTrajectories; 

    public:
        YamlDescriptor(int nHorizons, int nDrones, int nSubgoals): nHorizons(nHorizons), nDrones(nDrones), nSubgoals(nSubgoals) {}
        
        void setTimesArray(vector<vector<double> > timesArray) {
            this->timesArray = timesArray;
        }

        vector<vector<double> > getTimesArray() {
            return timesArray;
        }

        

};

#endif