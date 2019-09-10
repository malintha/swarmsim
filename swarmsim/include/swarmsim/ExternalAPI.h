#include <iostream>
#include "nav_msgs/Odometry.h"
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"

using namespace std;
using namespace Eigen;

enum APIType {
    MAVROS = 0, DJI
};

class ExternalAPI {
    public:
    ExternalAPI(int apiType, int droneId);
    virtual bool initAPI();
    virtual bool armDrone(bool arm);
    virtual bool TOL(bool takeoff);
    bool sendSetPoint(int derivative);

    string getLocalPositionTopic();
    string getGlobalPositionTopic();
    string getSetPointTopic();

    void getLocalPosition();
    void getGlobalPosition();
    bool getIsReady();
    void setDroneState(int state);
    
    private:
        int droneState;
        bool isReady; 
        int droneId;
        int apiType;
        ros::Subscriber globalPositionSub;
        ros::Subscriber localPositionSub;
        Vector3d localPos;
        Vector3d globalPos;
        Vector3d initGlobalPos;
        float yaw;


};