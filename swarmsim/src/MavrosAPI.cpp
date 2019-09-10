#include "MavrosAPI.h"

MavROSAPI::MavROSAPI(const ros::NodeHandle &nh, int droneId);
{
    ExternalAPI(APIType::MAVROS, droneId);
    this->droneId = droneId;
    this->setInitValues = false;
    //register the subscribers/publishers
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
    yaw = getRPY(msg->pose.pose.orientation)[2];
}

void MavROSAPI::positionGlobalCB(const sensor_msgs::NavSatFixConstPtr &msg) {
    globalPos << msg->latitude, msg->longitude, msg->altitude;
    if(!setInitValues) {
        initGlobalPos = globalPos;
    }
}

Vector3d MavROSAPI::getLocalPosition() {
    return localPos;
}

Vector3d MavROSAPI::getGlobalPosition() {
    return globalPos;
}

void MavROSAPI::mavrosStateCB(const mavros_msgs::StateConstPtr& msg) {
    bool guided = msg->guided;
    ROS_DEBUG_STREAM("Mavros Guided: "<<guided<<" Drone id: "<<this->id);
    if(guided) {
        isReady = true;
        this->mavrosStateSub.shutdown();
        ROS_DEBUG_STREAM("Drone: " << id << " Init position local: "
                            << curr_pos_local[0] << " " << curr_pos_local[1]
                            << " " << curr_pos_local[2]);
        ROS_DEBUG_STREAM("Drone: "
                            << id << " Init position global: " << curr_pos_global[0]
                            << " " << curr_pos_global[1] << " " << curr_pos_global[2]);
    }
}

void MavROSAPI::gazeboStateCB(const gazebo_msgs::ModelStatesConstPtr& msg) {
    if(droneState == States::Armed) {
        ros::V_string gazeboElementsArray = msg->name;
        stringstream drone_ss;
        drone_ss << "iris_"<<this->id;
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