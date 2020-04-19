#include "Visualize.h"

Visualize::Visualize(ros::NodeHandle nh, string worldframe, int ndrones):nh(nh), worldframe(worldframe), ndrones(ndrones) {
    this->initPaths();
    this->markerPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

void Visualize::initPaths() {
    //marker for the robot trajectories
    for(int i=0;i<this->ndrones;i++)  {
        visualization_msgs::Marker m;
        m.header.stamp = ros::Time::now();
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.header.frame_id=this->worldframe;
        m.id = i;
        m.color.r=0; 
        m.color.g=0; 
        m.color.b=1;
        m.color.a = 1;
        m.scale.x = 0.05;
        // m.scale.y = 0.01;
        // m.scale.z = 0.01;
        m.pose.orientation.w = 1;
        this->markerVec.push_back(m);
    }
}

void Visualize::addToPaths(vector<Trajectory> trajs) {
    for(int i=0; i < trajs.size(); i++) {
        Trajectory traj = trajs[i];
        ROS_DEBUG_STREAM("position list size: "<<traj.pos.size() << " " << traj.pos[0][0] << " " << traj.pos[0][1] << " " << traj.pos[0][2]);

        for (int p = 0; p < traj.pos.size(); p++) {
            geometry_msgs::Point pt;
            pt.x = traj.pos[p][0];
            pt.y = traj.pos[p][1];
            pt.z = traj.pos[p][2];
            this->markerVec[i].points.push_back(pt);
        }
    }
}

void Visualize::draw() {
    for(int i=0;i<this->ndrones;i++) {
        markerPub.publish(this->markerVec[i]);
    }
}