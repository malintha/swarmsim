#include "ExternalAPI.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/UInt8.h"

class DJIAPI:public ExternalAPI {
    public:
    DJIAPI(const ros::NodeHandle &nh);

    bool obtainControl();
    bool armDrone(bool arm) override;
    bool TOL(bool takeoff, double takeoffHeight) override;
    bool sendSetPoint(geometry_msgs::PoseStamped pose) override;
    Vector3d getLocalWaypoint(Vector3d waypoint) override;

    bool setLocalOrigin();

    private:
    ros::ServiceClient authorityService;
    ros::ServiceClient localPosReference;
    ros::NodeHandle nh;
    ros::Publisher posSetPointPub;
    ros::ServiceClient taskServiceCl;


    void positionGlobalCB(const sensor_msgs::NavSatFixConstPtr& msg);
    void positionLocalCB(const geometry_msgs::PointStamped::ConstPtr& msg); 

    bool M100monitoredTakeoff(double toHeight);
    bool takeoff_land(int task);
    uint8_t flight_status = 255;
    ros::Subscriber flightStatusSub;
    void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);

};