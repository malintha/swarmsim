#include "MavrosAPI.h"
#include "mavros_msgs/SetMode.h"
#include "utils.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"

MavROSAPI::MavROSAPI(const ros::NodeHandle &n, int droneId):nh(n)
{
    ExternalAPI(APIType::MAVROS, droneId);
    this->setInitValues = false;
    this->setReady = false;
    
    //register the subscribers/publishers
    string localPositionTopic = getLocalPositionTopic();
    string globalPositionTopic = getGlobalPositionTopic();
    string mavrosStateTopic = "";
    localPositionSub =
        nh.subscribe(localPositionTopic, 10, &MavROSAPI::positionLocalCB, this);
    globalPositionSub =
        nh.subscribe(globalPositionTopic, 10, &MavROSAPI::positionGlobalCB, this);
    mavrosStateSub = nh.subscribe(mavrosStateTopic, 10, &MavROSAPI::mavrosStateCB, this);
    gazeboStateSub = nh.subscribe("/gazebo/model_states", 10, &MavROSAPI::gazeboStateCB, this);
    posSetPointPub =
        nh.advertise<geometry_msgs::PoseStamped>(getSetPointTopic(), 10);
    ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states", ros::Duration(5));
}

void MavROSAPI::positionLocalCB(const nav_msgs::Odometry::ConstPtr &msg) {
    geometry_msgs::Point pos = msg->pose.pose.position;
    localPos << pos.x, pos.y, pos.z;
    yaw = simutils::getRPY(msg->pose.pose.orientation)[2];
}

void MavROSAPI::positionGlobalCB(const sensor_msgs::NavSatFixConstPtr &msg) {
    globalPos << msg->latitude, msg->longitude, msg->altitude;
    if(!setInitValues) {
        initGlobalPos = globalPos;
    }
}

void MavROSAPI::mavrosStateCB(const mavros_msgs::StateConstPtr& msg) {
    bool guided = msg->guided;
    ROS_DEBUG_STREAM("Mavros Guided: "<<guided<<" Drone id: "<<droneId);
    if(!setReady && guided) {
        setReady = true;
        ready(true);
        this->mavrosStateSub.shutdown();
        ROS_DEBUG_STREAM("Drone: " << droneId << " Init position local: "
                            << localPos[0] << " " << localPos[1]
                            << " " << localPos[2]);
        ROS_DEBUG_STREAM("Drone: "
                            << droneId << " Init position global: " << globalPos[0]
                            << " " << globalPos[1] << " " << globalPos[2]);
    }
}

void MavROSAPI::gazeboStateCB(const gazebo_msgs::ModelStatesConstPtr& msg) {
    if(state == States::Armed) {
        ros::V_string gazeboElementsArray = msg->name;
        stringstream drone_ss;
        drone_ss << "iris_"<<droneId;
        string drone_str = drone_ss.str();
        for(int i=0;i<gazeboElementsArray.size();i++) {
            string key = gazeboElementsArray[i];
            if(drone_str.compare(key) == 0) {
                this->gazeboElementIdx = i;
                geometry_msgs::Pose robot_pose = msg->pose[gazeboElementIdx];
                initGazeboPos << robot_pose.position.x, robot_pose.position.y, robot_pose.position.z;
                ROS_DEBUG_STREAM("Init gazebo pose recorded");
                gazeboStateSub.shutdown();
            }
        }
    }
}

string MavROSAPI::getMavrosStateName() {
    std::stringstream ss;
    ss << "/" << droneId << "/mavros/state";
    std::string mavrosStateTopic = ss.str();
    return mavrosStateTopic;
}

bool MavROSAPI::armDrone(bool arm) {
    stringstream ss_arm;
    ss_arm << "/" << droneId << "/mavros/cmd/arming";
    string arm_service = ss_arm.str();
    ros::ServiceClient arming_cl =
            nh.serviceClient<mavros_msgs::CommandBool>(arm_service);
    mavros_msgs::CommandBool srv;
    srv.request.value = arm;
    ROS_DEBUG_STREAM("Waiting for arm service " << arm_service);
    arming_cl.waitForExistence();
    if (arming_cl.call(srv)) {
        ROS_DEBUG_STREAM("Arm request sent");
    } else {
        ROS_ERROR_STREAM("Arm request failed for drone: " << droneId);
    }
    arm ? state = States::Armed : state = States::Ready;
}

bool MavROSAPI::TOL(bool takeoff) {
    std::string serviceId;
    takeoff ? serviceId = "takeoff" : serviceId = "land";
    stringstream ss_to;
    ss_to << "/" << droneId << "/mavros/cmd/" << serviceId;
    string to_service = ss_to.str();
    ros::ServiceClient to_cl =
            nh.serviceClient<mavros_msgs::CommandTOL>(to_service);
    mavros_msgs::CommandTOL srv_takeoff;

    if (takeoff) {
        srv_takeoff.request.latitude = initGlobalPos[0];
        srv_takeoff.request.longitude = initGlobalPos[1];
        srv_takeoff.request.altitude = initGlobalPos[2] + takeoffHeight;
        ROS_DEBUG_STREAM("Drone: " << droneId << " taking off at " << initGlobalPos[0]
                                   << " " << initGlobalPos[1] << " "
                                   << initGlobalPos[2]);

    } else {
        srv_takeoff.request.latitude = globalPos[0];
        srv_takeoff.request.longitude = globalPos[1];
        srv_takeoff.request.altitude = initGlobalPos[2] - localPos[2];
        ROS_DEBUG_STREAM("Drone is landing");
    }

    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = M_PI / 2;
    ROS_DEBUG_STREAM("Waiting for TOL service " << to_service);
    to_cl.waitForExistence();
    if (to_cl.call(srv_takeoff)) {
        ROS_DEBUG_STREAM("TOL request sent" << to_service);
    } else {
        ROS_ERROR_STREAM("TOL request failed " << to_service);
    }
}

bool MavROSAPI::setMode(string mode) {
    stringstream ss;
    ss << "/" << droneId << "/mavros/set_mode";
    string smService = ss.str();
    ros::ServiceClient smClient =
            nh.serviceClient<mavros_msgs::SetMode>(smService);
    mavros_msgs::SetMode setMode;
    setMode.request.custom_mode = mode;
    smClient.waitForExistence();

    // need to have some setpoints already in the queue to change to OFFBOARD mode
    if (mode.compare("OFFBOARD") == 0) {
        ROS_DEBUG_STREAM(
                "OFFBOARD called. Publishing current position to the queue");
        geometry_msgs::PoseStamped setpoint;
        setpoint.pose.position.x = localPos[0];
        setpoint.pose.position.y = localPos[1];
        setpoint.pose.position.z = localPos[2];
        for (int i = 0; i < 10; i++) {
            sendSetPoint(setpoint);
        }
    }

    if (smClient.call(setMode)) {
        ROS_DEBUG_STREAM("Set mode request sent " << mode << " " << smService);
    } else {
        ROS_DEBUG_STREAM("Set mode failed " << smService);
    }
}

Vector3d MavROSAPI::getLocalWaypoint(Vector3d waypoint) {
    return waypoint - initGazeboPos;
}

bool MavROSAPI::sendSetPoint(geometry_msgs::PoseStamped setPoint) {
    if (state == States::Autonomous) {
        this->posSetPointPub.publish(setPoint);
    }
}