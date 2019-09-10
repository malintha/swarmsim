#include "ExternalAPI.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/NavSatFix.h"
#include "mavros_msgs/State.h"

class MavROSAPI : public ExternalAPI {
    public:
    MavROSAPI(const ros::NodeHandle &nh, int droneId);
    string getMavrosStateName();
    bool initAPI() override;

    private:
    int droneId;
    bool setInitValues;
    bool guided;
    Vector3d initGazeboPos;

    ros::Subscriber mavrosStateSub;
    ros::Subscriber gazeboStateSub;
    ros::Publisher posSetPointPub;

    void gazeboStateCB(const gazebo_msgs::ModelStatesConstPtr& msg);
    void positionGlobalCB(const sensor_msgs::NavSatFixConstPtr& msg);
    void positionLocalCB(const nav_msgs::OdometryConstPtr& msg); 
    void mavrosStateCB(const mavros_msgs::StateConstPtr& msg);

};
