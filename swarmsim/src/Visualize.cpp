#include "Visualize.h"
#include "geometry_msgs/Point.h"

Visualize::Visualize(ros::NodeHandle nh, int nDrones) {
    this->marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    comMarker.header.stamp = ros::Time::now();
    comMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    comMarker.action = visualization_msgs::Marker::ADD;
    comMarker.header.frame_id="ground";
    comMarker.id = 100;
    comMarker.color.r=1; 
    comMarker.color.g=0; 
    comMarker.color.b=0;
    comMarker.color.a = 1;
    comMarker.scale.x = 0.05;
    comMarker.scale.y = 0.05;
    comMarker.scale.z = 0.05;
    comMarker.pose.orientation.w = 1;
}

void Visualize::draw(vector<Trajectory> trs) {
    for(int i=0;i<trs.size();i++) {
        std::vector<Eigen::Vector3d> wpts = trs[i].pos;
        for(int k=0;k<wpts.size();k++) {
            Eigen::Vector3d p = wpts[k];
            geometry_msgs::Point mp;
            mp.x = p[0];
            mp.y = p[1];
            mp.z = p[2];
            this->comMarker.points.push_back(mp);
        }
    marker_pub.publish(comMarker);
    }
}
