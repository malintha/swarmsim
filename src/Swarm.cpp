#include <iostream>
#include <ros/console.h>
#include "include/Swarm.h"
#include "include/state.h"
#include "include/utils.h"
#include <thread>
#include <chrono>
#include <stdexcept>

using namespace std;

Swarm::Swarm(const ros::NodeHandle &n, double frequency, int n_drones, bool fileLoad)
        : frequency(frequency), n_drones(n_drones), nh(n), fileLoad(fileLoad) {
    state = States::Idle;
    phase = Phases::Planning;
    horizonId = 0;
    yaml_fpath = "/home/malintha/drone_demo/install/share/swarmsim/launch/traj_data/goals.yaml";
    planningPhase = new SimplePlanningPhase(n_drones, frequency, yaml_fpath);

    planningInitialized = false;
    optimizingInitialized = false;
    for (int i = 0; i < n_drones; i++) {
        Drone *drone = new Drone(i, nh);
        dronesList.push_back(drone);
    }

    //loading the full trajectories from files
    if (fileLoad) {
        vector<Trajectory> trajectories = simutils::loadTrajectoriesFromFile(n_drones, nh, true);
        for (int i = 0; i < n_drones; i++) {
            Trajectory traj = trajectories[i];
            dronesList[i]->pushTrajectory(traj);
        }
    }

        //performing online trajectory optimization
    else {
        planningPhase->doPlanning(horizonId++);
        vector<Trajectory> trl = planningPhase->getPlanningResults();
        ROS_DEBUG_STREAM("Retrieved the initial planning results. Size: " << trl[0].pos.size());
        try {
            horizonLen = trl[0].pos.size();
        }
        catch (const length_error &le) {
            ROS_ERROR_STREAM("Error in retrieving the results from the future");
            return;
        }
        for (int i = 0; i < n_drones; i++) {
            dronesList[i]->pushTrajectory(trl[i]);
        }
    }
}

void Swarm::iteration(const ros::TimerEvent &e) {
    switch (state) {
        case States::Idle:
            checkSwarmForStates(States::Ready);
            break;

        case States::Ready:
            armDrones(true);
            checkSwarmForStates(States::Armed);
            break;

        case States::Armed:
            TOLService(true);
            checkSwarmForStates(States::Autonomous);
            break;

        case States::Autonomous:
            if (!fileLoad) {
                performPhaseTasks();
            }
            sendPositionSetPoints();
            checkSwarmForStates(States::Reached);
            break;

        case States::Reached:
            TOLService(false);
            break;

        default:
            break;
    }
}

void Swarm::run(float frequency) {
    this->frequency = frequency;
    ros::Timer timer = nh.createTimer(ros::Duration(1 / frequency),
                                      &Swarm::iteration, this);
    ros::spin();
}

void Swarm::setState(int state) {
    this->state = state;
    ROS_DEBUG_STREAM("Set swarm state: " << state);
}

void Swarm::checkSwarmForStates(int state) {
    bool swarmInState = true;
    for (int i = 0; i < n_drones; i++) {
        bool swarmInStateTemp;
        dronesList[i]->getState() == state ? swarmInStateTemp = true : swarmInStateTemp = false;
        swarmInState = swarmInState && swarmInStateTemp;
    }

    if (swarmInState) {
        setState(state);
    }
}

void Swarm::armDrones(bool arm) {
    for (int i = 0; i < n_drones; i++) {
        this->dronesList[i]->arm(arm);
    }
}

void Swarm::TOLService(bool takeoff) {
    for (int i = 0; i < n_drones; i++) {
        this->dronesList[i]->TOLService(takeoff);
    }
}

void Swarm::sendPositionSetPoints() {
    int execPointer;
    for (int i = 0; i < n_drones; i++) {
        execPointer = this->dronesList[i]->executeTrajectory();
    }
    if (!fileLoad) {
        setSwarmPhase(execPointer);
    }
}

/**
 * todo: change these ratios if one wants to use receding horizon planning.
 * eg: plan again when progress is 0.5 if the execution horizon = 0.5*planning horizon
*/
void Swarm::setSwarmPhase(int execPointer) {
    double progress = (double) execPointer / horizonLen;
    if (progress < 0.6) {
        if (progress == 0) {
            ROS_DEBUG_STREAM(
                    "Resetting planning and execution flags. exec: " << execPointer << " progress: " << progress);
            planningInitialized = false;
            executionInitialized = false;
        }
        phase = Phases::Planning;
    } else {
        phase = Phases::Execution;
    }
}

void Swarm::performPhaseTasks() {
    if (phase == Phases::Planning && !planningInitialized) {
        //initialize the external operations such as slam or task assignment
        try {
            planningPhase->doPlanning(horizonId);
            if (++horizonId > planningPhase->nHorizons) {
                executionInitialized = true;
            }
        }
        catch (runtime_error &e) {
            ROS_WARN_STREAM(e.what());
            planningPhase->planning_t->join();
            //set executionInitialized to true. So then it won't expect a value for the future.
            executionInitialized = true;
        }
        planningInitialized = true;
    } else if (phase == Phases::Execution && !executionInitialized) {
        //get the optimized trajectories from planningPhase and push them to the drones
        vector<Trajectory> results = planningPhase->getPlanningResults();
        ROS_DEBUG_STREAM("Optimization results retrieved");
        for (int i = 0; i < n_drones; i++) {
            dronesList[i]->pushTrajectory(results[i]);
        }
        executionInitialized = true;
    }
}

void Swarm::setWaypoints(vector<Trajectory> wpts, vector<double> tList) {
    if (phase == Phases::Planning) {
        this->wpts = wpts;
        this->tList = tList;
    } else {
        ROS_WARN_STREAM("Swarm is not in the planning phase. Waypoints rejected");
    }
}
