#include "utils.h"
#include <yaml.h>
#include <regex>
#include <ros/console.h>
#include <fstream>
#include <tuple>

using namespace Eigen;

namespace simutils {

    void
    processYamlFile(char *fPath, int horizon_id, int &horizons, vector<Trajectory> &goalPoints) {
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

        int subgoal_id = 0;
        int drone_id = 0;
        bool drone_node = false;
        bool horizon_node = false;
        bool subgoal_node = false;
        bool subgoals_node = false;
        bool horizons_node = false;
        int subgoals = 0;
        bool times_node = false;
        bool timeset_node = false;
        int timeset_id = 0;

        vector<double> pos_sequence;
        vector<Trajectory> tr_list;
        vector<double> t_list;

        const std::regex drone_regex("drone(\\d)");
        const std::regex horizon_regex("horizon(\\d)");
        const std::regex subgoal_regex("subgoal");
        const std::regex subgoals_regex("subgoals");
        const std::regex horizons_regex("horizons");
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

                case YAML_MAPPING_START_EVENT:
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
                        ssub_match sub_match = pieces_match[1];
                        std::string piece = sub_match.str();
                        drone_id = std::stoi(piece);
                        continue;
                    } else if (std::regex_match(s, pieces_match, horizon_regex)) {
                        horizon_node = true;
                        ssub_match sub_match = pieces_match[1];
                        std::string piece = sub_match.str();
                        subgoal_id = std::stoi(piece);
                        continue;
                    } else if (std::regex_match(s, subgoals_regex)) {
                        subgoals_node = true;
                        continue;
                    } else if (std::regex_match(s, horizons_regex)) {
                        horizons_node = true;
                        continue;
                    } else if (std::regex_match(s, times_regex)) {
                        times_node = true;
                        continue;
                    } else if (std::regex_match(s, pieces_match, timeset_regex)) {
                        timeset_node = true;
                        ssub_match sub_match = pieces_match[1];
                        std::string piece = sub_match.str();
                        timeset_id = std::stoi(piece);
                        continue;
                    }
                    if (subgoals_node) {
                        subgoals = std::stoi(s);
                        subgoals_node = false;
                    }

                    if (horizons_node) {
                        horizons = std::stoi(s);
                        horizons_node = false;
                    }

                    if (timeset_node && times_node) {
                        if (timeset_id == horizon_id) {
                            t_list.push_back(std::stod(s));
                        }
                    }

                    if (horizon_node && subgoal_node) {
                        if (subgoal_id == horizon_id) {

                            if (pos_sequence.size() < 3 * subgoals) {
                                pos_sequence.push_back(std::stod(s));
                            }
                            if (pos_sequence.size() == 3 * subgoals) {
                                Eigen::Vector3d p;
                                Trajectory tr;
                                for (int i = 0; i < pos_sequence.size(); i++) {
                                    int d = i % 3;
                                    if (d <= 2) {
                                        p[d] = pos_sequence[i];
                                    }
                                    if (d == 2) {
                                        tr.pos.push_back(p);
                                    }
                                }
                                tr.tList = t_list;
                                goalPoints.push_back(tr);
                                pos_sequence.clear();
                            }
                        }
                    }
                    break;
            }
            if (event.type != YAML_STREAM_END_EVENT) {
                yaml_event_delete(&event);
            }
        } while (event.type != YAML_STREAM_END_EVENT);
        yaml_event_delete(&event);


        if (subgoal_id < horizon_id && goalPoints.size() == 0) {
            throw range_error("Horizon does not exists");
        }
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
}