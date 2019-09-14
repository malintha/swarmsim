#include <iostream>
#include <ros/console.h>
#include "Swarm.h"
#include "state.h"
#include <thread>
#include <chrono>
#include <stdexcept>
#include <std_msgs/Int8.h>

using namespace std;

Swarm::Swarm(const ros::NodeHandle &n, double frequency, int n_drones, string& trajDir)
        : frequency(frequency), n_drones(n_drones), nh(n) {
    initVariables();
    vector<Trajectory> trajectories = simutils::loadTrajectoriesFromFile(n_drones, nh, trajDir);
    for (int i = 0; i < n_drones; i++) {
        Trajectory traj = trajectories[i];
        dronesList[i]->pushTrajectory(traj);
    }
    swarmStatePub = nh.advertise<std_msgs::Int8>("swarm/state", 100, false);
}

Swarm::Swarm(const ros::NodeHandle &n, double frequency, int n_drones, string& trajDir, string& yamlFileName) :
        frequency(frequency), n_drones(n_drones), nh(n) {
        predefined = false;
        stringstream ss;
        ss << trajDir<<yamlFileName;
        string yamlFilePath = ss.str();
        swarmStatePub = nh.advertise<std_msgs::Int8>("swarm/state", 100, false);
        executionInitialized = false;
        // visualizer = new Visualize(n, n_drones);

    try {
        if (yamlFilePath.empty()) {
            throw runtime_error("YAML file path is not provided. Exiting.");
        }
        initVariables();
        planningPhase = new SimplePlanningPhase(n, n_drones, frequency, yamlFilePath);
        planningPhase->doPlanning(horizonId++);
        vector<Trajectory> trl = planningPhase->getPlanningResults();
        // visualizer->draw(trl);
        ROS_DEBUG_STREAM("Retrieved the initial planning results. Size: " << trl[0].pos.size());
        horizonLen = trl[0].pos.size();
        for (int i = 0; i < n_drones; i++) {
            dronesList[i]->pushTrajectory(trl[i]);
        }
    }
    catch (const length_error &le) {
        ROS_ERROR_STREAM("Error in retrieving the results from the future");
        return;
    }
    catch (const runtime_error &re) {
        ROS_ERROR_STREAM("Error initializing the swarm. "<<re.what());
        return;
    }
}

void Swarm::initVariables() {
    planExecutionRatio = 0.8;
    state = States::Idle;
    phase = Phases::Planning;
    horizonId = 0;
    planningInitialized = false;
    for (int i = 0; i < n_drones; i++) {
        Drone *drone = new Drone(i, nh);
        dronesList.push_back(drone);
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
            if (!predefined) {
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

void Swarm::run(float frequency_) {
    this->frequency = frequency_;
    ros::Timer timer = nh.createTimer(ros::Duration(1 / frequency),
                                      &Swarm::iteration, this);
    ros::spin();
}

void Swarm::setState(int state_) {
    this->state = state_;
    ROS_DEBUG_STREAM("Set swarm state: " << state);
}

void Swarm::checkSwarmForStates(int state_) {
    bool swarmInState = true;
    for (int i = 0; i < n_drones; i++) {
        bool swarmInStateTemp;
        dronesList[i]->getState() == state_ ? swarmInStateTemp = true : swarmInStateTemp = false;
        swarmInState = swarmInState && swarmInStateTemp;
    }

    if (swarmInState) {
        setState(state_);
    }
    std_msgs::Int8 msg;
    msg.data = this->state;
    swarmStatePub.publish(msg);

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
    int execPointer = 0;
    for (int i = 0; i < n_drones; i++) {
        execPointer = this->dronesList[i]->executeTrajectory();
    }
    if (!predefined) {
        setSwarmPhase(execPointer);
    }
}

/**
 * todo: change these ratios if one wants to use receding horizon planning.
 * eg: plan again when progress is 0.5 if the execution horizon = 0.5*planning horizon
*/
void Swarm::setSwarmPhase(int execPointer) {
    double progress = (double) execPointer / horizonLen;
    if (progress > planExecutionRatio) {
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
            if(horizonId < planningPhase->nHorizons)
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
        // visualizer->draw(results);
        ROS_DEBUG_STREAM("Optimization results retrieved");
        for (int i = 0; i < n_drones; i++) {
            dronesList[i]->pushTrajectory(results[i]);
        }
        executionInitialized = true;
    }
}

void Swarm::setWaypoints(vector<Trajectory> wpts_, vector<double> tList_) {
    if (phase == Phases::Planning) {
        this->wpts = move(wpts_);
        this->tList = move(tList_);
    } else {
        ROS_WARN_STREAM("Swarm is not in the planning phase. Waypoints rejected");
    }
}
