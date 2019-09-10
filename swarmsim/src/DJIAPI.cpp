#include "DJIAPI.h"
#include "dji_sdk/dji_sdk.h"

using namespace Eigen;

DJIAPI::DJIAPI(const ros::nodeHandle &nh) {
    ExternalAPI(APIType::DJI);
    string authorityServiceName = getAuthorityServiceName();
    authorityService = nh.subscribe(authorityServiceName);
    string refPointServiceName = getSetPointTopic();

}

//getAuthority
bool DJIAPI::initAPI() {
    bool obtainedControl = obtainControl();
    if (obtainedControl) {
        setInitLocalPosition();
    }
}

string DJIAPI::getAuthorityServiceName() {
    return "dji_sdk/sdk_control_authority";
}

bool DJIAPI::obtainControl() {
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable=1;
    authorityService.call(authority);
    if(!authority.response.result) {
        ROS_ERROR("obtain control failed!");
        return false;
    }
    return true;
}

bool DJIAPI::setInitLocalPosition() {
    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    set_local_pos_reference.call(localPosReferenceSetter);
    return localPosReferenceSetter.response.result;
}
