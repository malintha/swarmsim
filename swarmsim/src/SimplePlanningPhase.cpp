#include <algorithm>
#include "SimplePlanningPhase.h"

SimplePlanningPhase::SimplePlanningPhase() = default;

SimplePlanningPhase::SimplePlanningPhase(int nDrones, double frequency, string yamlFpath) : PlanningPhase(nDrones,
                                                                                                          frequency) {
    this->yamlFpath = move(yamlFpath);
    ros::NodeHandle nh;
    localGoalsSub = nh.subscribe("local_way_points", 10, &SimplePlanningPhase::localGoalsCB, this);
}

void SimplePlanningPhase::doPlanning(int horizonId) {
    auto sharedP = make_shared<promise<vector<Trajectory> > >();
    fut = sharedP->get_future();
    auto doPlanningExpr = [horizonId, this, sharedP]() {
        try {
            discreteWpts = this->getDiscretePlan(horizonId);
        }
        catch (range_error &e) {
            ROS_ERROR_STREAM("Error occurred while planning!");
            return;
        }
       
        vector<Trajectory> smoothTrajs = computeSmoothTrajectories();
        try {
            sharedP->set_value_at_thread_exit(smoothTrajs);
        }
        catch (std::future_error &e) {
            ROS_ERROR_STREAM("Caught a future_error while fulfilling the promise\"" << e.what());
        }
        doneInitPlanning = true;
    };
    planning_t = new thread(doPlanningExpr);
}

/**
 * This needs to query from the visual slam and get the discreet waypoints
 * set nHorizons to a large number
*/
vector<Trajectory> SimplePlanningPhase::getDiscretePlan(int horizonId) {
    vector<Trajectory> planningResults;
    try {
        planningResults = getExecutionTrajectory();
        ROS_DEBUG_STREAM("Planning results size: "<<planningResults.size() << "Horizon id: "<<horizonId);
        nHorizons = 1;
    }
    catch (range_error &e) {
        ROS_WARN_STREAM(e.what() << " " << horizonId);
        throw range_error(e.what());
    }
    return planningResults;
}

void SimplePlanningPhase::localGoalsCB(const geometry_msgs::PoseArray& msg) {
    ROS_DEBUG_STREAM("Retrieved a new path");
    nHorizons++;
    this->path = msg;
}

vector<Trajectory> SimplePlanningPhase::getExecutionTrajectory() {
    //limit the # of waypoints from the discreet path to 4
    int exTrajectoryLength = 3;
    vector<Trajectory> exTrajectory;
    Trajectory tr;
    for(int i=0;i<exTrajectoryLength;i++) {
        Eigen::Vector3d pos, rpy;
        geometry_msgs::Pose wp = this->path.poses[i];
        pos << wp.position.x, wp.position.y, wp.position.z;
        if(i==0) {
            ROS_DEBUG_STREAM("### Starting WP: "<<pos[0]<<" "<<pos[1]<<" "<<pos[2]);
        }
        rpy = simutils::getRPY(wp.orientation);
        tr.pos.push_back(pos);
        tr.rpy.push_back(rpy);
    }
    exTrajectory.push_back(tr);
    return exTrajectory;
}