#include "Drone.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "ros/console.h"
#include "DJIAPI.h"
#include "MavrosAPI.h"
#include "utils.h"
#include <tf/tf.h>

using namespace std;

Drone::Drone(int id, const ros::NodeHandle &n, int droneType) : id(id), nh(n) {
    ROS_DEBUG_STREAM("Initializing drone " << id);
    this->wptsFollowing = true;
    this->apiType = droneType;
    if(this->apiType == APIType::DJIType) {
        this->extAPI = new DJIAPI(n);
    }
    else if (this->apiType == APIType::MAVROS) {
        this->extAPI = new MavROSAPI(n, id);
    }
    setReady = false;
    takeoffHeight = 1;
    execPointer = 0;
    trajectoryId = 0;
    extAPI->setTakeoffHeight(takeoffHeight);
}

int Drone::getState() {
    return extAPI->getState();
}

void Drone::setState(int state) {
    extAPI->setState(state);
}

void Drone::arm(bool arm) {
    extAPI->armDrone(arm);
}

void Drone::TOLService(bool takeoff) {
    // std::cout<<"setinitvalues: "<<extAPI->setInitValues<<std::endl;
    if (extAPI->getState() == States::Armed && takeoff && extAPI->setInitValues) {
        extAPI->TOL(true, takeoffHeight);
        extAPI->setState(States::Takingoff);
    } else if (extAPI->getState() == States::Takingoff) {
        if (extAPI->getLocalPosition()[2] >= takeoffHeight - 0.1) {
            extAPI->setState(States::Autonomous);
            if(this->apiType == APIType::MAVROS) {
                if(wptsFollowing) {
                    static_cast<MavROSAPI*>(extAPI)->setMode("OFFBOARD");
                    // std::cout<<"STABILIZED"<<std::endl;
                } else {
                    static_cast<MavROSAPI*>(extAPI)->setMode("OFFBOARD");
                }
            }
        }
    } else if (extAPI->getState() == States::Reached && !takeoff) {
        extAPI->TOL(false, 0);
        extAPI->setState(States::Idle);
    }

}

void Drone::setTrajectory(Trajectory trajectory) {
    execPointer = 0;
    this->trajectory = trajectory;
}

int Drone::executeTrajectory() {
    if (extAPI->getState() == States::Autonomous) {
        Vector3d waypoint_temp, waypoint;
        Vector3d attitude;
        //notReachedEnd
        if (execPointer < trajectory.pos.size() - 1) {
            waypoint_temp = trajectory.pos[execPointer++];
            waypoint = extAPI->getLocalWaypoint(waypoint_temp);
        }
            //reachedEnd and moreTrajectoriesAvailable
        else if ((trajectoryId < TrajectoryList.size() - 1) && (execPointer == trajectory.pos.size() - 1)) {
            ROS_DEBUG_STREAM("Setting next trajectory for drone: " << id <<" "<<trajectory.pos.size());
            Trajectory nextTraj = TrajectoryList[++trajectoryId];
            setTrajectory(nextTraj);
            waypoint = trajectory.pos[execPointer];
            attitude = trajectory.rpy[execPointer];
            ROS_DEBUG_STREAM("set next wpts for drone: " << id << " " << waypoint[0] << " " << waypoint[1] << " "
                                                         << waypoint[2]);
        }
            //reachedEnd and noMoreTrajectories
        else {
            ROS_DEBUG_STREAM_ONCE("No more trajectories. Waiting...");
            // static_cast<MavROSAPI*>(extAPI)->setMode("AUTO.LOITER");
            // extAPI->setState(States::Reached);
            waypoint = trajectory.pos[execPointer - 1];
        }
        geometry_msgs::PoseStamped setpoint;
        setpoint.pose.position.x = waypoint[0];
        setpoint.pose.position.y = waypoint[1];
        setpoint.pose.position.z = waypoint[2];
        tf::Quaternion qt;
        double yaw = attitude[2];
        qt.setRPY(0, 0, yaw);
        // ROS_DEBUG_STREAM("yaw: "<<yaw);
        setpoint.pose.orientation.w = qt.w();
        setpoint.pose.orientation.x = qt.x();
        setpoint.pose.orientation.y = qt.y();
        setpoint.pose.orientation.z = qt.z();

        extAPI->sendSetPoint(setpoint);
    }
    return execPointer;
}

void Drone::pushTrajectory(Trajectory trajectory) {
    TrajectoryList.push_back(trajectory);
    //setting the initial trajectory
    if (TrajectoryList.size() == 1) {
        setTrajectory(TrajectoryList[trajectoryId]);
    }
}

/**
 * Return true if the distance is within 10cm
 */
bool Drone::reachedGoal(geometry_msgs::PoseStamped setPoint) {
    Eigen::Vector3d setP;
    setP << setPoint.pose.position.x, setPoint.pose.position.y,
            setPoint.pose.position.z;
    if (simutils::getEucDistance(extAPI->getLocalPosition(), setP) < 0.2) {
        // setState(States::Reached);
        return true;
    }
    return false;
}

void Drone::addWaypoints(vector<geometry_msgs::Pose> newWpts) {
    for(geometry_msgs::Pose p : newWpts) {
        this->wptsList.push_back(p);
    }
}

void Drone::executeMission() {
    currentTarget = wptsList.front();
    geometry_msgs::PoseStamped sp;
    sp.pose = currentTarget;
    if(!reachedGoal(sp)) {
        extAPI->sendSetPoint(sp);
    }
    else if(reachedGoal(sp)) {
        std::cout<<"reached goal "<<sp.pose.position.x<<" "<<sp.pose.position.y<<std::endl;
        if(wptsList.size() > 0) {
            wptsList.pop_front();
        }
        else {
            TOLService(false);
        }
    }
}
