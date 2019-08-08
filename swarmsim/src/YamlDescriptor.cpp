#include "YamlDescriptor.h"

YamlDescriptor::YamlDescriptor() = default;

void YamlDescriptor::setHorizons(int nHorizons) {
    this->nHorizons = nHorizons;
}

void YamlDescriptor::setDrones(int nDrones) {
    this->nDrones = nDrones;
}

void YamlDescriptor::setSubGoals(int nSubgoals) {
    this->nSubgoals = nSubgoals;
}

int YamlDescriptor::getSubGoals() {
    return this->nSubgoals;
}

void YamlDescriptor::setTimesArray(vector<HorizonTimes> timesArray) {
    this->timesArray = timesArray;
}

vector<HorizonTimes> YamlDescriptor::getTimesArray() {
    return timesArray;
}

void YamlDescriptor::setDronesTrajectories(vector<DroneTrajectory> dronesTr) {
    this->droneTrajectories = dronesTr;
}

vector<DroneTrajectory> YamlDescriptor::getdroneTrajectories() {
    return droneTrajectories;
}

int YamlDescriptor::getHorizons() {
    return this->nHorizons;
}

int YamlDescriptor::getDrones() {
    return nDrones;
}