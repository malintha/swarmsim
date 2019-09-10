#include "ExternalAPI.h"

class DJIAPI:public ExternalAPI {
    public:
    DJIAPI(const ros::NodeHandle &nh);

    string getAuthorityServiceName();

    bool initAPI();
    bool obtainControl();
    bool setInitLocalPosition();
    Vector3d getLocalWaypoint(Vector3d waypoint);

    private:
    ros::ServiceClient authorityService;
    ros::ServiceClient localPosReference;


};