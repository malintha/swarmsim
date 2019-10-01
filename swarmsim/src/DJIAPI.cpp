#include "DJIAPI.h"
#include "dji_sdk/dji_sdk.h"
#include "sensor_msgs/Joy.h"
#include "utils.h"
#include "dji_sdk/DroneArmControl.h"
#include "dji_sdk/DroneTaskControl.h"
#include "dji_sdk/SDKControlAuthority.h"
#include "dji_sdk/SetLocalPosRef.h"
#include <tf2/LinearMath/Quaternion.h>

using namespace Eigen;
// using namespace DJI::OSDK;

DJIAPI::DJIAPI(const ros::NodeHandle &n) : ExternalAPI(APIType::DJIType, 0), nh(n)
{

    string localPositionTopic = getLocalPositionTopic();
    string globalPositionTopic = getGlobalPositionTopic();

    globalPositionSub =
        nh.subscribe(globalPositionTopic, 10, &DJIAPI::positionGlobalCB, this);
    localPositionSub =
        nh.subscribe(localPositionTopic, 10, &DJIAPI::positionLocalCB, this);
    ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &DJIAPI::flight_status_callback, this);
    posSetPointPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
    taskServiceCl = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");

    string refPointServiceName = getSetPointTopic();
    bool obtainedControl = obtainControl();
    setLocalOrigin();
    if (obtainedControl)
    {
        ready(true);
    }
}

bool DJIAPI::armDrone(bool arm)
{
    arm ? state = States::Armed : state = States::Ready;
}

bool DJIAPI::TOL(bool takeoff, double takeoffHeight)
{
    ROS_DEBUG_STREAM("Waiting for DJI task service ");
    taskServiceCl.waitForExistence();
    dji_sdk::DroneTaskControl taskReq;
    if (takeoff)
    {
        ROS_DEBUG_STREAM("Taking off");
        M100monitoredTakeoff(takeoffHeight);
    }
    else
    {
        ROS_DEBUG_STREAM("Landing");
        takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND);
    }
}

bool DJIAPI::sendSetPoint(geometry_msgs::PoseStamped setPoint)
{
    sensor_msgs::Joy setP;
    double targetX = setPoint.pose.position.x;
    double targetY = setPoint.pose.position.y;

    // ROS_DEBUG_STREAM("curr: "<<localPos[0]<<" , "<<localPos[1]<<" target: "<<targetX<<" , "<<targetY);
    setP.axes.push_back(targetX - localPos[0]);
    setP.axes.push_back(targetY - localPos[1]);
    setP.axes.push_back(setPoint.pose.position.z);
    std::cout<<setPoint.pose.orientation.w<<std::endl;

    Vector3d rpy = simutils::getRPY(setPoint.pose.orientation);
    setP.axes.push_back(rpy[2]);
    posSetPointPub.publish(setP);
}

void DJIAPI::positionLocalCB(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    geometry_msgs::Point pos = msg->point;
    localPos << pos.x, pos.y, pos.z;
}

void DJIAPI::positionGlobalCB(const sensor_msgs::NavSatFixConstPtr &msg)
{
    globalPos << msg->latitude, msg->longitude, msg->altitude;
}

void DJIAPI::flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

bool DJIAPI::obtainControl()
{
    ros::ServiceClient authorityService = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 1;
    ROS_DEBUG_STREAM("Waiting for DJI arm service ");
    authorityService.waitForExistence();
    authorityService.call(authority);
    if (!authority.response.result)
    {
        ROS_ERROR("obtain control failed!");
        return false;
    }
    return true;
}

bool DJIAPI::setLocalOrigin()
{
    ros::ServiceClient setLocalPosRefCl = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");
    ROS_DEBUG_STREAM("Waiting for DJI arm service ");
    setLocalPosRefCl.waitForExistence();
    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    setLocalPosRefCl.call(localPosReferenceSetter);
    return localPosReferenceSetter.response.result;
}

Vector3d DJIAPI::getLocalWaypoint(Vector3d waypoint)
{
    return waypoint;
}

bool DJIAPI::M100monitoredTakeoff(double toHeight)
{
    ros::Time start_time = ros::Time::now();
    float home_altitude = globalPos[2];
    if (!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
    {
        return false;
    }
    ros::Duration(0.01).sleep();

    while (ros::Time::now() - start_time < ros::Duration(5))
    {
        ros::spinOnce();
    }

    if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR) {
        return false;
    }

    return true;
}

bool DJIAPI::takeoff_land(int task)
{
    dji_sdk::DroneTaskControl droneTaskControl;
    droneTaskControl.request.task = task;
    taskServiceCl.call(droneTaskControl);
    if (!droneTaskControl.response.result)
    {
        ROS_ERROR("takeoff_land fail");
        return false;
    }
    return true;
}