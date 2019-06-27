#include "utils.h"
#include <yaml.h>
#include <regex>
#include <ros/console.h>
#include <fstream>
#include <tuple>

using namespace Eigen;

namespace simutils {

vector<Trajectory> processYamlFile(char* fPath, int horizon_id) {
    FILE *fh = fopen(fPath, "r");
    yaml_parser_t parser;
    yaml_token_t token;
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
    bool subgoalset_node = false;
    bool subgoal_node = false;
    bool subgoals_node = false;
    int subgoals = 0;
    bool times_node = false;
    bool timeset_node = false;
    int timeset_id = 0;

    vector<double> pos_sequence;
    vector<Trajectory> tr_list;
    vector<double> t_list;
  
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
                if(timeset_node)
                    timeset_node = false;
            break;

            case YAML_MAPPING_START_EVENT:
            break;
            
            case YAML_MAPPING_END_EVENT:
                if(subgoalset_node && drone_node) {
                    subgoalset_node = false;
                }
                else if(drone_node) {
                    drone_node = false;
                }
            break;

            case YAML_SCALAR_EVENT:
                char *c = (char *)event.data.scalar.value;
                string s = std::string(c);
                const std::regex drone_regex("drone(\\d)");
                const std::regex subgoalset_regex("subgoalset(\\d)");
                const std::regex subgoal_regex("subgoal");
                const std::regex subgoals_regex("subgoals");
                const std::regex times_regex("times");
                const std::regex timeset_regex("timeset(\\d)");
                std::smatch pieces_match;

                if(std::regex_match(s, pieces_match,drone_regex)) {
                    drone_node = true;
                    ssub_match sub_match = pieces_match[1];
                    std::string piece = sub_match.str();
                    drone_id = std::stoi(piece);
                    continue;
                }
                else if(std::regex_match(s, pieces_match,subgoalset_regex)) {
                    subgoalset_node = true;
                    ssub_match sub_match = pieces_match[1];
                    std::string piece = sub_match.str();
                    subgoal_id = std::stoi(piece);
                    continue;
                }
                else if(std::regex_match(s, subgoals_regex)) {
                    subgoals_node = true;
                    continue;
                }
                else if(std::regex_match(s, times_regex)) {
                    times_node = true;
                    continue;
                }
                else if(std::regex_match(s, pieces_match, timeset_regex)) {
                    timeset_node = true;
                    ssub_match sub_match = pieces_match[1];
                    std::string piece = sub_match.str();
                    timeset_id = std::stoi(piece);
                    continue;
                }

                if(subgoals_node) {
                    subgoals = std::stoi(s);
                    subgoals_node = false;
                }

                if(timeset_node && times_node) {
                    if(timeset_id == horizon_id) {
                        t_list.push_back(std::stod(s));
                    }
                }

                if(subgoalset_node && subgoal_node) {
                    if(subgoal_id == horizon_id) {

                        if(pos_sequence.size() < 3*subgoals) {
                            pos_sequence.push_back(std::stod(s));
                        }
                        if(pos_sequence.size() == 3*subgoals) {
                            Eigen::Vector3d p;
                            Trajectory tr;
                            for(int i=0;i<pos_sequence.size();i++) {
                                int d = i%3;
                                if(d <= 2) {
                                    p[d] = pos_sequence[i];
                                }
                                if(d==2) {
                                    tr.pos.push_back(p);
                                }
                            }
                            tr.tList = t_list;
                            tr_list.push_back(tr);
                            pos_sequence.clear();
                        }
                    }
                }
        break;
        }
        if (event.type != YAML_STREAM_END_EVENT) {
            yaml_event_delete(&event);
        }
    } 
    while (event.type != YAML_STREAM_END_EVENT);
    yaml_event_delete(&event);

    if(subgoal_id < horizon_id && tr_list.size() == 0) {
        cout<<"horz_id"<<subgoal_id<<" tr_list"<<tr_list.size()<<endl;
        throw range_error("Horizon does not exists");
    }

    return tr_list;
    }


    std::vector<double> loadTimesFromFile(ros::NodeHandle &nh) {
        std::vector<double> tList;
        std::string filePath;
        if(nh.getParam("/swarmsim/trajDir",filePath)) {
            std::stringstream ss;
            ss << filePath <<"tList"<<".txt";
            ROS_DEBUG_STREAM("Loading times from file "<<ss.str());
            std::ifstream tstream(ss.str());
            double t;
            Trajectory traj;
            while(tstream >> t) {
                tList.push_back(t);
            }   
        }
        ROS_DEBUG_STREAM("Times loaded from file");
        return tList;
    }

    std::vector<Trajectory> loadTrajectoriesFromFile(int n_drones, ros::NodeHandle &nh, bool fullTrajecory){
        std::vector<Trajectory> trajList;
        std::string filePath;
        std::string prefix;
        fullTrajecory ? prefix = "pos_" : prefix = "goals_";

        if(nh.getParam("/swarmsim/trajDir",filePath)) {
            for(int i=0;i<n_drones;i++) {
            std::stringstream ss;
            ss << filePath <<prefix<<i<<".txt";
            ROS_DEBUG_STREAM("Loading trajectories from file "<<ss.str());
            std::ifstream posStream(ss.str());
            std::string posLine;
            double x, y, z;
            Trajectory traj;
            while(posStream >> x >> y >> z) {
                Eigen::Vector3d pos;
                pos << x, y, z;
                traj.pos.push_back(pos);
            }
            trajList.push_back(traj);
            }
        }
        ROS_DEBUG_STREAM("Trajectories loaded from file");
        return trajList;
        }


    void reSizeMat(vector<int> *A, int prevDim, int newDim) {
    vector<int> B(newDim * newDim);
    fill(B.begin(), B.begin() + newDim * newDim, 0);
    for (int i = 0; i < newDim * newDim; i++) {
        int r = i / newDim;
        int c = i % newDim;
        if (r < prevDim && c < prevDim) {
        B[i] = A->at(r * prevDim + c);
        }
    }
    *A = B;
    }

    void printmat(vector<int> *H) {
    int n = sqrt(H->size());
    for (int i = 0; i < H->size(); i++) {
        int c = i % n;
        int r = i / n;
        cout << H->at(i) << " ";
        if (c == n - 1) {
        cout << endl;
        }
    }
    cout << endl;
    }

    void blockDiag(vector<int> *H, real_t *Hn, int HnRows) {
    int HnLen = HnRows * HnRows;
    int currDim = sqrt(H->size());
    int nblocks = currDim / HnRows;
    int newDim = HnRows * (nblocks + 1);
    reSizeMat(H, currDim, newDim);
    for (int i = 0; i < (newDim * newDim); i++) {
        int c = i % newDim;
        int r = i / newDim;
        if (r > currDim - 1 || c > currDim - 1) {
        if (r < currDim && c > currDim - 1) {
            H->at(i) = 0;
        } else if (r > currDim - 1 && c < currDim) {
            H->at(i) = 0;
        } else {
            int r_n = r % HnRows;
            int c_n = c % HnRows;
            H->at(i) = (Hn[r_n * HnRows + c_n]);
        }
        }
    }
    }

}