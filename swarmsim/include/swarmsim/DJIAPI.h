#include "ExternalAPI.h"

class DJIAPI:public ExternalAPI {
    public:
    DJIAPI(const ros::NodeHandle &nh);

    string getAuthorityServiceName();

    bool initAPI() override;
    bool obtainControl();
    bool setInitLocalPosition();
    
    private:
    ros::ServiceClient authorityService;
    ros::ServiceClient localPosReference;


};