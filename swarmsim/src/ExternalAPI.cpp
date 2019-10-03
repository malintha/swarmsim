#include "ExternalAPI.h"

ExternalAPI::ExternalAPI() {}

ExternalAPI::ExternalAPI(int APIType, int droneId) {
    this->droneId = droneId;
    this->apiType = APIType;
    this->state = States::Idle;
}

string ExternalAPI::getGlobalPositionTopic() {
    string topic;
    if(apiType == APIType::DJIType) {
        topic = "dji_sdk/gps_position";
    } 
    else {
        stringstream ss;
        ss << "/" << this->droneId << "/mavros/global_position/global";
        topic = ss.str();
    }
    return topic;
}

string ExternalAPI::getLocalPositionTopic() {
    string topic;
    if(apiType == APIType::DJIType) {
        topic = "dji_sdk/local_position";
    }
    else {
        stringstream ss;
        ss << "/" << this->droneId << "/mavros/global_position/local";
        topic = ss.str();
    }
    return topic;
}

string ExternalAPI::getSetPointTopic() {
    string setPointName;
    if(apiType == APIType::DJIType) {
        setPointName = "dji_sdk/set_local_pos_ref";
    }
     else {
         stringstream ss;
         ss << "/" << this->droneId << "/mavros/setpoint_position/local";
         setPointName = ss.str();
     }
    return setPointName;
}

void ExternalAPI::ready(bool ready) {
    if(ready) {
        setState(States::Ready);
        ROS_DEBUG_STREAM("MAVROS READY. Drone: " << droneId);
    }
}

int ExternalAPI::getState() {
    return this->state;
}

void ExternalAPI::setState(int state) {
    this->state = state;
}

Vector3d ExternalAPI::getLocalPosition() {
    return localPos;
}

Vector3d ExternalAPI::getGlobalPosition() {
    return globalPos;
}

void ExternalAPI::setTakeoffHeight(double takeoffHeight) {
    this->takeoffHeight = takeoffHeight;
}

