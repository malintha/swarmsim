#include "ExternalAPI.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/NavSatFix.h"
#include "mavros_msgs/State.h"
#include "geometry_msgs/Pose.h"

class MavROSAPI : public ExternalAPI {
    public:
    MavROSAPI(const ros::NodeHandle &n, int droneId);
    Vector3d initGazeboPos;
    string getMavrosStateName();
    
    bool armDrone(bool arm) override;
    bool TOL(bool takeoff, double takeoffHeight) override;
    bool sendSetPoint(geometry_msgs::PoseStamped pose) override;
    Vector3d getLocalWaypoint(Vector3d waypoint) override;

    bool setMode(string mode);

    private:

    bool setReady;
    int droneId;
    // bool setInitValues;
    bool guided;
    ros::NodeHandle nh;
    ros::Subscriber mavrosStateSub;
    ros::Subscriber gazeboStateSub;
    ros::Publisher posSetPointPub;
    int gazeboElementIdx;
    

    void gazeboStateCB(const gazebo_msgs::ModelStatesConstPtr& msg);
    void positionGlobalCB(const sensor_msgs::NavSatFixConstPtr& msg);
    void positionLocalCB(const nav_msgs::OdometryConstPtr& msg); 
    void mavrosStateCB(const mavros_msgs::StateConstPtr& msg);
    

};
