#include "ExternalAPI.h"

string ExternalAPI::getLocalPositionTopic() {
    string topic;
    if(apiType == APITYPE::DJI) {
        topic = "dji_sdk/local_position";
    } 
    else {
        stringstream ss;
        ss << this->droneId << "/mavros/global_position/global";
        topic = ss.str();
    }
    droneState = States::Idle;
    return topic;
}

string ExternalAPI::getGlobalPositionTopic() {
    string topic;
    if(apiType == APITYPE::DJI) {
        topic = "dji_sdk/gps_position";
    } 
    else {
        stringstream ss;
        ss << this->droneId << "/mavros/global_position/local";
        topic = ss.str();
    }
    return topic;
}

string ExternalAPI::getSetPointTopic() {
    string setPointName;
    if(apiType == APITYPE::DJI) {
        setPointName = "dji_sdk/set_local_pos_ref";
    }
     else {
         stringstream ss;
         ss << "/" << this->id << "/mavros/setpoint_position/local";
         setPointName = ss.str();
     }
    return setPointName;
}

bool ExternalAPI::getIsReady() {
    return isReady;
}

void ExternalAPI::setdroneState(int state) {
    droneState = state;
}