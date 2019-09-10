#include "utils.h"
#include <yaml.h>
#include <regex>
#include <ros/console.h>
#include <fstream>
#include <tuple>
#include "tf/tf.h"


using namespace Eigen;

namespace simutils {

    void processYamlFile(char *fPath, YamlDescriptor &yamlDescriptor) {
        FILE *fh = fopen(fPath, "r");
        yaml_parser_t parser;
        yaml_event_t event;

        if (!yaml_parser_initialize(&parser)) {
            std::cout << "Failed to initialize parser!\n" << std::endl;
        }
        if (fh == NULL) {
            std::cout << "Failed to open file!\n" << std::endl;
        }

        yaml_parser_set_input_file(&parser, fh);

        parseYamlHeader(fPath, yamlDescriptor);

        bool drone_node = false;
        bool horizon_node = false;
        bool subgoal_node = false;
        bool horizons_node = false;
        bool times_node = false;
        bool timeset_node = false;
        bool movingThresholdNode = false;
        bool hoveringThresholdNode = false;

        vector<double> pos_sequence;
        int nHorizons;
        int nDrones;
        int subgoals;
        vector<DroneTrajectory> dronesTrajList;
        vector<double> timeSequence;
        vector<HorizonTimes> horizonsTimes;

        nHorizons = yamlDescriptor.getHorizons();
        subgoals = yamlDescriptor.getSubGoals();
        nDrones = yamlDescriptor.getDrones();

        const std::regex drone_regex("drone(\\d)");
        const std::regex horizon_regex("horizon(\\d)");
        const std::regex times_regex("times");
        const std::regex timeset_regex("timeset(\\d)");
        std::smatch pieces_match;

        do {
            if (!yaml_parser_parse(&parser, &event)) {
                printf("Parser error %d\n", parser.error);
                exit(EXIT_FAILURE);
            }

            switch (event.type) {
                case YAML_SEQUENCE_START_EVENT:
                    subgoal_node = true;
                    break;

                case YAML_SEQUENCE_END_EVENT:
                    subgoal_node = false;
                    if (timeset_node)
                        timeset_node = false;
                    break;

                case YAML_MAPPING_END_EVENT:
                    if (horizon_node && drone_node) {
                        horizon_node = false;
                    } else if (drone_node) {
                        drone_node = false;
                    }
                    break;

                case YAML_SCALAR_EVENT:
                    char *c = (char *) event.data.scalar.value;
                    string s = std::string(c);
                    if (std::regex_match(s, pieces_match, drone_regex)) {
                        drone_node = true;
                        continue;
                    } else if (std::regex_match(s, pieces_match, horizon_regex)) {
                        horizon_node = true;
                        continue;
                    } else if (std::regex_match(s, times_regex)) {
                        times_node = true;
                        continue;
                    } else if (std::regex_match(s, pieces_match, timeset_regex)) {
                        timeset_node = true;
                        continue;
                    } else if(s.compare("movingThreshold") == 0) {
                        movingThresholdNode = true;
                        continue;
                    } else if(s.compare("hoveringThreshold") == 0) {
                        hoveringThresholdNode = true;
                        continue;
                    }
                    if(movingThresholdNode) {
                        yamlDescriptor.setMovingThreshold(std::stod(s));
                        movingThresholdNode = false;
                    }
                    if(hoveringThresholdNode) {
                        yamlDescriptor.setHoveringThreshold(std::stod(s));
                        hoveringThresholdNode = false;
                    }
                    if (timeset_node && times_node) {
                        timeSequence.push_back(std::stod(s));
                    }
                    if (horizon_node && subgoal_node) {
                        pos_sequence.push_back(std::stod(s));
                    }
                    break;
            }
            if (event.type != YAML_STREAM_END_EVENT) {
                yaml_event_delete(&event);
            }
        } while (event.type != YAML_STREAM_END_EVENT);
        yaml_event_delete(&event);
                
        for(int h=0;h<nHorizons;h++) {
            HorizonTimes horzTimes;
            for(int i=0;i<subgoals-1;i++) {
                double t = timeSequence[i + h*(subgoals-1)];
                horzTimes.times.push_back(t);
            }
            horizonsTimes.push_back(horzTimes);
        }

        for(int i=0;i<nDrones;i++) {
            DroneTrajectory droneTr;
            for(int h=0;h<nHorizons;h++) {
                Trajectory tr;
                int startIdx = 3*subgoals*nHorizons*i + 3*subgoals*h;
                int endIdx = startIdx + 3*subgoals;
                Vector3d p;
                for(int idx=startIdx;idx<endIdx;idx++) {
                    int dim = idx%3;
                    if(dim <= 2) {
                        p[dim] = pos_sequence[idx];
                    }
                    if(dim == 2) {
                        tr.pos.push_back(p);
                    }
                }
                droneTr.horzTrajList.push_back(tr);
            }
            dronesTrajList.push_back(droneTr);
        }

        yamlDescriptor.setDronesTrajectories(dronesTrajList);
        yamlDescriptor.setTimesArray(horizonsTimes);
        ROS_DEBUG_STREAM("Finished parsing the yaml body information");
    }

    std::vector<double> loadTimesFromFile(ros::NodeHandle &nh) {
        std::vector<double> tList;
        std::string filePath;
        if (nh.getParam("/swarmsim/trajDir", filePath)) {
            std::stringstream ss;
            ss << filePath << "tList" << ".txt";
            ROS_DEBUG_STREAM("Loading times from file " << ss.str());
            std::ifstream tstream(ss.str());
            double t;
            Trajectory traj;
            while (tstream >> t) {
                tList.push_back(t);
            }
        }
        ROS_DEBUG_STREAM("Times loaded from file");
        return tList;
    }

    std::vector<Trajectory> loadTrajectoriesFromFile(int n_drones, ros::NodeHandle &nh, const string filePath) {
        std::vector<Trajectory> trajList;
        std::string prefix;

        for (int i = 0; i < n_drones; i++) {
            std::stringstream ss;
            ss << filePath << prefix << i << ".txt";
            ROS_DEBUG_STREAM("Loading trajectories from file " << ss.str());
            std::ifstream posStream(ss.str());
            std::string posLine;
            double x, y, z;
            Trajectory traj;
            while (posStream >> x >> y >> z) {
                Eigen::Vector3d pos;
                pos << x, y, z;
                traj.pos.push_back(pos);
            }
            trajList.push_back(traj);
        }
        ROS_DEBUG_STREAM("Trajectories loaded from file");
        return trajList;
    }

    int getGazeboModelId(std::vector<std::string> modelNames, string elementName) {
        for(int i=0;i<modelNames.size();i++) {
            std::string key = modelNames[i];
            if(key.compare(elementName) == 0) {
                return i;
            }
        }
        return -1;
    }

    void parseYamlHeader(char *fPath, YamlDescriptor &yamlDescriptor) {
        FILE *fh = fopen(fPath, "r");
        yaml_parser_t parser;
        yaml_event_t event;
        if (!yaml_parser_initialize(&parser)) {
            std::cout << "Failed to initialize parser!\n" << std::endl;
        }
        if (fh == NULL) {
            std::cout << "Failed to open file!\n" << std::endl;
        }
        yaml_parser_set_input_file(&parser, fh);
        bool nDronesNode;
        bool nHorizonsNode;
        bool nSubGoalsNode;
        int nDrones;
        int nHorizons;
        int nSubgoals;

        do {
            if (!yaml_parser_parse(&parser, &event)) {
                printf("Parser error %d\n", parser.error);
                exit(EXIT_FAILURE);
            }

            switch (event.type) {
                case YAML_SCALAR_EVENT:
                    char *c = (char *) event.data.scalar.value;
                    string s = std::string(c);

                    if(s.compare("drones") == 0) {
                        nDronesNode = true;
                        continue;
                    }
                    else if(s.compare("horizons") == 0) {
                        nHorizonsNode = true;
                        continue;
                    }
                    else if(s.compare("subgoals") == 0) {
                        nSubGoalsNode = true;
                        continue;
                    }
                    if (nSubGoalsNode) {
                        ROS_DEBUG_STREAM("nSubGoalsNode "<<s);
                        nSubgoals = std::stoi(s);
                        nSubGoalsNode = false;
                    }
                    else if (nHorizonsNode) {
                        ROS_DEBUG_STREAM("nHorizonsNode "<<s);
                        nHorizons = std::stoi(s);
                        nHorizonsNode = false;
                    }
                    else if(nDronesNode) {
                        ROS_DEBUG_STREAM("nDronesNode "<<s);
                        nDrones = std::stoi(s);
                        nDronesNode = false;
                    }
                    break;
            }
            if (event.type != YAML_STREAM_END_EVENT) {
                yaml_event_delete(&event);
            }
        } while (event.type != YAML_STREAM_END_EVENT);

        yamlDescriptor.setDrones(nDrones);
        yamlDescriptor.setHorizons(nHorizons);
        yamlDescriptor.setSubGoals(nSubgoals);

        yaml_event_delete(&event);
        ROS_DEBUG_STREAM("Done parsing the yaml header information");

    }

    vector<Trajectory> getHorizonTrajetories(int horizonId, YamlDescriptor yamlDescriptor) {
        vector<DroneTrajectory> droneTrajectories = yamlDescriptor.getdroneTrajectories();
        vector<Trajectory> trs;
        for(int i=0;i<yamlDescriptor.getDrones();i++) {
            DroneTrajectory dtr = droneTrajectories[i];
            Trajectory tr = dtr.horzTrajList[horizonId];
            vector<HorizonTimes> hz_t = yamlDescriptor.getTimesArray(); 
            tr.tList = hz_t[horizonId].times;
            trs.push_back(tr);
        }
        return trs;
    }

    Vector3d getRPY(geometry_msgs::Quaternion orientation) {
        tfScalar roll, pitch, yaw;
        Vector3d rpy;
        tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z,
                                    orientation.w))
                .getRPY(roll, pitch, yaw);
        rpy << roll, pitch, yaw;
        return rpy;
    }

    float getEucDistance(Eigen::Vector3d p1, Eigen::Vector3d p2) {
        return (p1 - p2).norm();
    }

}