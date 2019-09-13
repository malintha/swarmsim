#include "Visualize.h"
#include "geometry_msgs/Point.h"

Visualize::Visualize(int nDrones) {
    ros::NodeHandle nh;
    this->marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    for(int i=0;i<nDrones;i++) {
        visualization_msgs::Marker m;
        m.header.frame_id="map";
        m.header.stamp = ros::Time::now();
        m.ns = "path";
        m.id = 100;

        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.color.r=1; 
        m.color.g=0; 
        m.color.b=0;
        m.color.a = 1;
        m.scale.x = 0.05;
        m.scale.y = 0.05;
        m.scale.z = 0.05;
        m.pose.orientation.x = 0.0;
        m.pose.orientation.y = 0.0;
        m.pose.orientation.z = 0.0;
        m.pose.orientation.w = 1.0;
        m.lifetime = ros::Duration();

        this->markerVec.push_back(m);
    }
}

void Visualize::draw(vector<Trajectory> trs) {
    ROS_DEBUG_STREAM("draw");
    vector<Eigen::Vector3d> pos = trs[0].pos;
    for(int k=0;k<pos.size();k++) {
        geometry_msgs::Point mp;
        mp.x = pos[k][0];
        mp.y = pos[k][1];
        mp.z = pos[k][2];
        this->markerVec[0].points.push_back(mp);
    }

    marker_pub.publish(markerVec[0]);
}
