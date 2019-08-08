#include <iostream>
#include "Trajectory.h"
#include <vector>
#include "DroneTrajectory.h"
#include "HorizonTimes.h"

using namespace std;

class YamlDescriptor{
    public:
        YamlDescriptor();
        void setHorizons(int nHorizons);
        void setDrones(int nDrones);
        void setSubGoals(int nSubgoals);
        int getSubGoals();
        void setTimesArray(vector<HorizonTimes> timesArray);
        vector<HorizonTimes> getTimesArray();
        void setDronesTrajectories(vector<DroneTrajectory> dronesTr);
        vector<DroneTrajectory> getdroneTrajectories();
        int getHorizons();
        int getDrones();

    private:
        int nHorizons;
        int nSubgoals;
        int nDrones;
        vector<HorizonTimes> timesArray;
        double passThresholdMoving;
        double passThresholdHover;
        vector<DroneTrajectory> droneTrajectories; 
};