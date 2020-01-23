#include <algorithm>
#include "SimplePlanningPhase.h"
#include <geometry_msgs/Pose.h>

SimplePlanningPhase::SimplePlanningPhase() = default;

SimplePlanningPhase::SimplePlanningPhase(int nDrones, double frequency, string yamlFpath) : PlanningPhase(nDrones,
                                                                                                          frequency) {
    this->yamlFpath = move(yamlFpath);
    this->nDrones = nDrones;
    robotPathsSub = nh.subscribe("/robotPaths", 10, &SimplePlanningPhase::robotPathsCB, this);
    recievedPaths = false;
}

void SimplePlanningPhase::doPlanning(int horizonId) {

    ROS_DEBUG_STREAM("Computing trajectories");
    auto sharedP = make_shared<promise<vector<Trajectory> > >();
    fut = sharedP->get_future();
    auto doPlanningExpr = [horizonId, this, sharedP]() {
        try {
            discreteWpts = this->recievedTrajs;
        }
        catch (range_error &e) {
            ROS_ERROR_STREAM("Error occurred while planning!");
            return;
        }
        bool initialQP;
        bool lastQP;
        horizonId == 0 ? initialQP = true : initialQP = false;
        horizonId == nHorizons ? lastQP = true : lastQP = false;
        
        vector<Trajectory> smoothTrajs = computeSmoothTrajectories(initialQP, lastQP);
        try {
            sharedP->set_value_at_thread_exit(smoothTrajs);
        }
        catch (std::future_error &e) {
            ROS_ERROR_STREAM("Caught a future_error while fulfilling the promise\"" << e.what());
        }

        doneInitPlanning = true;
        recievedPaths = false;
    };
    planning_t = new thread(doPlanningExpr);
}

void SimplePlanningPhase::robotPathsCB(const geometry_msgs::PoseArrayConstPtr& msg) {
    ROS_DEBUG_STREAM("Robot paths retrieved. Size: " << msg->poses.size());
    this->recievedTrajs.clear();
    int size = msg->poses.size();
    int pathLength = size/nDrones;
    for(int i=0; i<nDrones; i++) {
        Trajectory tr;
        for(int j=i*pathLength;j<i*pathLength + pathLength; j++) {
            geometry_msgs::Pose p = msg->poses[j];
            Vector3d pos;
            pos << p.position.x, p.position.y, p.position.z;
            tr.pos.push_back(pos);
        }
        recievedTrajs.push_back(tr);
    }
    recievedPaths = true;
}

vector<Trajectory> SimplePlanningPhase::getDiscretePlan(int horizonId) {
    ROS_DEBUG_STREAM("## pathLength: "<<this->recievedTrajs.size());

    ROS_DEBUG_STREAM("YAML file path: " << yamlFpath);
    char cstr[yamlFpath.size() + 1];
    copy(yamlFpath.begin(), yamlFpath.end(), cstr);
    cstr[yamlFpath.size()] = '\0';
    vector<Trajectory> planningResults;
    bool initPlan = horizonId == 0;
    try {
        if(initPlan) {
            simutils::processYamlFile(cstr, yamlDescriptor);
        }
        ROS_DEBUG_STREAM("getTimesArray: "<<yamlDescriptor.getTimesArray()[1].times.size());
        planningResults = simutils::getHorizonTrajetories(horizonId, yamlDescriptor);
        nHorizons = yamlDescriptor.getHorizons();
        ROS_DEBUG_STREAM("planningResults: "<<planningResults.size());
    }
    catch (range_error &e) {
        ROS_WARN_STREAM(e.what() << " " << horizonId);
        throw range_error(e.what());
    }
    return planningResults;
}