#include "DJIAPI.h"
#include "dji_sdk/dji_sdk.h"
#include "sensor_msgs/Joy.h"
#include "utils.h"
#include "dji_sdk/DroneArmControl.h"
#include "dji_sdk/DroneTaskControl.h"
#include "dji_sdk/SDKControlAuthority.h"
#include "dji_sdk/SetLocalPosRef.h"

using namespace Eigen;
using namespace DJI::OSDK;

DJIAPI::DJIAPI(const ros::NodeHandle &n):nh(n) {
    ExternalAPI(APIType::DJIType, droneId);
    string localPositionTopic = getLocalPositionTopic();
    string globalPositionTopic = getGlobalPositionTopic();

    globalPositionSub =
        nh.subscribe(globalPositionTopic, 10, &DJIAPI::positionGlobalCB, this);
    localPositionSub =
        nh.subscribe(localPositionTopic, 10, &DJIAPI::positionLocalCB, this);
    posSetPointPub = nh.advertise<sensor_msgs::Joy>
        ("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
    taskServiceCl = nh.serviceClient<dji_sdk::DroneTaskControl>
        ("dji_sdk/drone_task_control");
    
    string refPointServiceName = getSetPointTopic();
    bool obtainedControl = obtainControl();
    if (obtainedControl) {
        if(setLocalOrigin()) {
            ready(true);
        }
    }
}

bool DJIAPI::armDrone(bool arm) {
    ros::ServiceClient arming_cl =
        nh.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/drone_arm_control");
    dji_sdk::DroneArmControl droneArmControl;
    droneArmControl.request.arm = arm;
    ROS_DEBUG_STREAM("Waiting for DJI arm service ");
    arming_cl.waitForExistence();
    arming_cl.call(droneArmControl);
    if(droneArmControl.response.result) {
        ROS_DEBUG_STREAM("Arm request sent");
        arm ? state = States::Armed : state = States::Ready;
    }
    else {
        ROS_ERROR_STREAM("Arm request failed for DJI");
    }
}

bool DJIAPI::TOL(bool takeoff) {
    ROS_DEBUG_STREAM("Waiting for DJI task service ");
    taskServiceCl.waitForExistence();
    dji_sdk::DroneTaskControl taskReq;
    if(takeoff) {
        taskReq.request.task = 4;
    }
    else {
        taskReq.request.task = 6;
    }
    
    taskServiceCl.call(taskReq);

    if(taskReq.response.result) {
        ROS_DEBUG_STREAM("TOL request sent to DJI");
    }
    else {
        ROS_ERROR_STREAM("TOL request for DJI failed");
    }
}

bool DJIAPI::sendSetPoint(geometry_msgs::PoseStamped setPoint) {
    sensor_msgs::Joy setP;
    setP.axes.push_back(setPoint.pose.position.x);
    setP.axes.push_back(setPoint.pose.position.y);
    setP.axes.push_back(setPoint.pose.position.z);
    Vector3d rpy = simutils::getRPY(setPoint.pose.orientation);
    setP.axes.push_back(rpy[2]);
    posSetPointPub.publish(setP);
} 

void DJIAPI::positionLocalCB(const geometry_msgs::PointStamped::ConstPtr& msg) {
    geometry_msgs::Point pos = msg->point;
    localPos << pos.x, pos.y, pos.z;
}


void DJIAPI::positionGlobalCB(const sensor_msgs::NavSatFixConstPtr &msg) {
    globalPos << msg->latitude, msg->longitude, msg->altitude;
}

bool DJIAPI::obtainControl() {
    ros::ServiceClient authorityService = nh.serviceClient<dji_sdk::SDKControlAuthority>
      ("dji_sdk/sdk_control_authority");
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable=1;
    ROS_DEBUG_STREAM("Waiting for DJI arm service ");
    authorityService.waitForExistence();
    authorityService.call(authority);
    if(!authority.response.result) {
        ROS_ERROR("obtain control failed!");
        return false;
    }
    return true;
}

bool DJIAPI::setLocalOrigin() {
    ros::ServiceClient setLocalPosRefCl = nh.serviceClient<dji_sdk::SetLocalPosRef> 
        ("dji_sdk/set_local_pos_ref");
    ROS_DEBUG_STREAM("Waiting for DJI arm service ");
    setLocalPosRefCl.waitForExistence();
    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    setLocalPosRefCl.call(localPosReferenceSetter);
    return localPosReferenceSetter.response.result;
}

Vector3d DJIAPI::getLocalWaypoint(Vector3d waypoint) {
    return waypoint;
}