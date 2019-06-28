#include "include/Drone.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/SetMode.h"
#include "ros/console.h"

using namespace std;

Drone::Drone(int id, const ros::NodeHandle &n) : id(id), nh(n) {
  ROS_DEBUG_STREAM("Initializing drone " << id);
  setState(States::Idle);
  takeoffHeight = 2.5;
  execPointer = 0;
  trajectoryId = 0;
  std::string globalPositionTopic = getPositionTopic("global");
  std::string localPositionTopic = getPositionTopic("local");
  std::string poseTopic = getPoseTopic();
  localPositionSub =
      nh.subscribe(localPositionTopic, 10, &Drone::positionLocalCB, this);
  globalPositionSub =
      nh.subscribe(globalPositionTopic, 10, &Drone::positionGlobalCB, this);
  std::string positionSetPointTopic = getLocalSetpointTopic("position");
  posSetPointPub =
      nh.advertise<geometry_msgs::PoseStamped>(positionSetPointTopic, 10);
}

int Drone::getState() { return state; }

void Drone::positionLocalCB(const nav_msgs::Odometry::ConstPtr &msg) {
  geometry_msgs::Point pos = msg->pose.pose.position;
  curr_pos_local << pos.x, pos.y, pos.z;
  yaw = getRPY(msg->pose.pose.orientation)[2];
  if (!setInitPosLocal) {
    init_pos_local = curr_pos_local;
    setInitPosLocal = true;
    ROS_DEBUG_STREAM("Drone: " << id << " Init position local: "
                               << init_pos_local[0] << " " << init_pos_local[1]
                               << " " << init_pos_local[2]);
    ready(setInitPosGlobal, setInitPosLocal);
  }
}

void Drone::positionGlobalCB(const sensor_msgs::NavSatFixConstPtr &msg) {
  curr_pos_global << msg->latitude, msg->longitude, msg->altitude;
  if (!setInitPosGlobal) {
    init_pos_global = curr_pos_global;
    setInitPosGlobal = true;
    ROS_DEBUG_STREAM("Drone: "
                     << id << " Init position global: " << init_pos_global[0]
                     << " " << init_pos_global[1] << " " << init_pos_global[2]);
    ready(setInitPosGlobal, setInitPosLocal);
  }
}

// void Drone::poseCB(const geometry_msgs::PoseStampedConstPtr &msg) {}

void Drone::arm(bool arm) {
  stringstream ss_arm;
  ss_arm << "/" << id << "/mavros/cmd/arming";
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
    ROS_ERROR_STREAM("Arm request failed for drone: " << id);
  }
  arm == true ? state = States::Armed : States::Ready;
}

void Drone::TOLService(bool takeoff) {
  if (state == States::Armed && takeoff) {
    callTOLService(true);
    setState(States::Takingoff);
  } else if (state == States::Takingoff) {
    if (curr_pos_local[2] >= takeoffHeight - 0.2) {
      setState(States::Autonomous);
      setMode("OFFBOARD");
    }
  } else if (state == States::Reached && !takeoff) {
    callTOLService(false);
    setState(States::Idle);
  }
}

void Drone::callTOLService(bool takeoff) {
  std::string serviceId;
  takeoff ? serviceId = "takeoff" : serviceId = "land";
  stringstream ss_to;
  ss_to << "/" << id << "/mavros/cmd/" << serviceId;
  string to_service = ss_to.str();
  ros::ServiceClient to_cl =
      nh.serviceClient<mavros_msgs::CommandTOL>(to_service);
  mavros_msgs::CommandTOL srv_takeoff;

  if (takeoff) {
    srv_takeoff.request.latitude = init_pos_global[0];
    srv_takeoff.request.longitude = init_pos_global[1];
    srv_takeoff.request.altitude = init_pos_global[2] + takeoffHeight;
    ROS_DEBUG_STREAM("Drone: " << id << " taking off at " << init_pos_global[0]
                               << " " << init_pos_global[1] << " "
                               << init_pos_global[2]);

  } else {
    srv_takeoff.request.latitude = curr_pos_global[0];
    srv_takeoff.request.longitude = curr_pos_global[1];
    srv_takeoff.request.altitude = init_pos_global[2] - curr_pos_local[2];
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

void Drone::setMode(std::string mode) {
  stringstream ss;
  ss << "/" << id << "/mavros/set_mode";
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
    setpoint.pose.position.x = curr_pos_local[0];
    setpoint.pose.position.y = curr_pos_local[1];
    setpoint.pose.position.z = curr_pos_local[2];
    for (int i = 0; i < 10; i++) {
      sendPositionSetPoint(setpoint);
    }
  }

  if (smClient.call(setMode)) {
    ROS_DEBUG_STREAM("Set mode request sent " << mode << " " << smService);
  } else {
    ROS_DEBUG_STREAM("Set mode failed " << smService);
  }
}

void Drone::sendPositionSetPoint(geometry_msgs::PoseStamped setPoint) {
  if (state == States::Autonomous) {
    posSetPointPub.publish(setPoint);
  }
}

void Drone::setTrajectory(Trajectory trajectory) {
  execPointer = 0;
  this->trajectory = trajectory;
}

int Drone::executeTrajectory() {
  if (state == States::Autonomous) {
    Vector3d waypoint;
    //notReachedEnd
    if (execPointer < trajectory.pos.size() - 1) {
      waypoint = trajectory.pos[execPointer++];
    }
    //reachedEnd and moreTrajectoriesAvailable
    else if((trajectoryId < TrajectoryList.size() - 1) && (execPointer == trajectory.pos.size() - 1)) {
      ROS_DEBUG_STREAM("Setting next trajectory for drone: "<<this->id);
      Trajectory nextTraj = TrajectoryList[++trajectoryId];
      setTrajectory(nextTraj);
      waypoint = trajectory.pos[execPointer];
      ROS_DEBUG_STREAM("set next wpts for drone: "<<id<<" "<<waypoint[0]<<" "<<waypoint[1]<<" "<<waypoint[2]);
      cout<<"execpointer: "<<execPointer<<endl;
    }
    //reachedEnd and noMoreTrajectories
    else {
      ROS_DEBUG_STREAM("No more trajectories. Setting state as Reached");
      setMode("AUTO.LOITER");
      setState(States::Reached);
      waypoint = trajectory.pos[execPointer - 1];
    }
    geometry_msgs::PoseStamped setpoint;
    setpoint.pose.position.x = waypoint[0];
    setpoint.pose.position.y = waypoint[1];
    setpoint.pose.position.z = waypoint[2];
    sendPositionSetPoint(setpoint);
  }
  return execPointer;
}

void Drone::pushTrajectory(Trajectory trajectory) {
  TrajectoryList.push_back(trajectory);
  //setting the initial trajectory
  if(TrajectoryList.size() == 1) {
    setTrajectory(TrajectoryList[trajectoryId]);
  }
}

std::string Drone::getPositionTopic(std::string locale) {
  std::stringstream ss;
  ss << "/" << this->id << "/mavros/global_position/" << locale;
  return ss.str();
}

std::string Drone::getPoseTopic() {
  std::stringstream ss;
  ss << "/" << this->id << "/mavros/local_position/pose";
  return ss.str();
}

Vector3d Drone::getRPY(geometry_msgs::Quaternion orientation) {
  tfScalar roll, pitch, yaw;
  Vector3d rpy;
  tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z,
                               orientation.w))
      .getRPY(roll, pitch, yaw);
  rpy << roll, pitch, yaw;
  return rpy;
}

void Drone::setState(int state) {
  this->state = state;
  ROS_DEBUG_STREAM("Drone: " << id << " Set drone state " << state);
}

void Drone::ready(bool setInitPosGlobal, bool setInitPosLocal) {
  if (setInitPosGlobal && setInitPosLocal) {
    setState(States::Ready);
  }
}

/**
 * Return true if the distance is within 10cm
 */
bool Drone::reachedGoal(geometry_msgs::PoseStamped setPoint) {
  Eigen::Vector3d setP;
  setP << setPoint.pose.position.x, setPoint.pose.position.y,
      setPoint.pose.position.z;
  // cout<<"eucledian dis: "<<getEucDistance(curr_pos_local, setP)<<endl;
  if (getEucDistance(curr_pos_local, setP) < 0.1) {
    setState(States::Reached);
    return true;
  }
  return false;
}

std::string Drone::getLocalSetpointTopic(std::string order) {
  std::stringstream ss;
  std::string postfix;
  if (order.compare("position") == 0) {
    postfix = "local";
  } else if (order.compare("velocity") == 0) {
    postfix = "cmd_vel";
  } else if (order.compare("accel") == 0) {
    postfix = "accel";
  }
  ss << "/" << this->id << "/mavros/setpoint_" << order << "/" << postfix;
  return ss.str();
}

float Drone::getEucDistance(Eigen::Vector3d p1, Eigen::Vector3d p2) {
  return (p1 - p2).norm();
}
