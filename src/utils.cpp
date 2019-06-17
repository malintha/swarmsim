#include "utils.h"
#include <yaml.h>
#include <regex>

using namespace std;

namespace simutils {

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


vector<Trajectory> getTrajectoryList(char* fPath, int horizon_id) {
    FILE *fh = fopen(fPath, "r");
  yaml_parser_t parser;
  yaml_token_t token;
  yaml_event_t event;

  if (!yaml_parser_initialize(&parser))
    std::cout << "Failed to initialize parser!\n" << std::endl;
  if (fh == NULL)
    std::cout << "Failed to open file!\n" << std::endl;

  yaml_parser_set_input_file(&parser, fh);

  int subgoal_id = 0;
  int drone_id = 0;
  int dim = 0;
  bool drone_node = false;
  bool subgoalset_node = false;
  bool subgoal_node = false;
  bool pushed_curr_sequence = false;

  vector<Trajectory> tr_list;
  Eigen::Vector3d pos;
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
      break;

    case YAML_MAPPING_START_EVENT:
      break;

    case YAML_MAPPING_END_EVENT:
      if (subgoalset_node && drone_node) {
        subgoalset_node = false;
      } else if (drone_node) {
        drone_node = false;
      }
      break;

    case YAML_SCALAR_EVENT:
      char *c = (char *)event.data.scalar.value;
      string s = std::string(c);
      const std::regex drone_regex("drone(\\d)");
      const std::regex subgoalset_regex("subgoalset(\\d)");
      const std::regex subgoal_regex("subgoal");
      std::smatch pieces_match;

      if (std::regex_match(s, pieces_match, drone_regex)) {
        drone_node = true;
        ssub_match sub_match = pieces_match[1];
        std::string piece = sub_match.str();
        drone_id = std::stoi(piece);
        continue;
      } else if (std::regex_match(s, pieces_match, subgoalset_regex)) {
        subgoalset_node = true;
        ssub_match sub_match = pieces_match[1];
        std::string piece = sub_match.str();
        subgoal_id = std::stoi(piece);
        continue;
      }

      if (subgoalset_node && subgoal_node) {
        if (subgoal_id == horizon_id) {
          pushed_curr_sequence = false;
          pos[dim] = std::stod(s);
          if (dim == 2) {
            Trajectory tr;
            Eigen::Vector3d p;
            for (int i = 0; i < 3; i++) {
              p[i] = pos[i];
            }
            tr.pos.push_back(p);
            tr_list.push_back(tr);
            dim = 0;
          } else {
            dim++;
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
  
  return tr_list;
}

}