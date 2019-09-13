#include <algorithm>
#include "SimplePlanningPhase.h"

SimplePlanningPhase::SimplePlanningPhase() = default;

SimplePlanningPhase::SimplePlanningPhase(ros::NodeHandle nh, int nDrones, double frequency, string yamlFpath) : PlanningPhase(nDrones,
                                                                                                          frequency) {
    this->yamlFpath = move(yamlFpath);
    
    // localGoalsSub = nh.subscribe("local_way_points", 10, &SimplePlanningPhase::localGoalsCB, this);
    nHorizons = 0;
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
    // geometry_msgs::PoseArray path;
    try {
        ROS_DEBUG_STREAM("Waiting for planning data to publish");
        this->path = *(ros::topic::waitForMessage<geometry_msgs::PoseArray>("local_way_points",ros::Duration(10)));
        
        nHorizons++;
        planningResults = getExecutionTrajectory();
        ROS_DEBUG_STREAM("Execution planning size: "<<planningResults[0].pos.size() << "Horizon id: "<<horizonId);
    }
    catch (range_error &e) {
        ROS_WARN_STREAM(e.what() << " " << horizonId);
        throw range_error(e.what());
    }
    catch(...){
        ROS_ERROR_STREAM("Caught unknown error while recieving the planning results");
    }
    return planningResults;
}

// void SimplePlanningPhase::localGoalsCB(const geometry_msgs::PoseArray& msg) {
//     ROS_DEBUG_STREAM("Retrieved a new path");
//     nHorizons++;
//     this->path = msg;
// }

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