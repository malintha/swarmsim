#include "ExternalAPI.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/NavSatFix.h"


class DJIAPI:public ExternalAPI {
    public:
    DJIAPI(const ros::NodeHandle &nh);

    bool obtainControl();
    bool armDrone(bool arm) override;
    bool TOL(bool takeoff) override;
    bool sendSetPoint(geometry_msgs::PoseStamped pose) override;


    bool setLocalOrigin();
    Vector3d getLocalWaypoint(Vector3d waypoint);

    private:
    ros::ServiceClient authorityService;
    ros::ServiceClient localPosReference;
    ros::NodeHandle nh;
    ros::Publisher posSetPointPub;
    ros::ServiceClient taskServiceCl;

    void positionGlobalCB(const sensor_msgs::NavSatFixConstPtr& msg);
    void positionLocalCB(const geometry_msgs::PointStamped::ConstPtr& msg); 


};